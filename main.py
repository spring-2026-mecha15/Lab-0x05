# Main control loop for dual-motor closed-loop control system.

# This module initializes hardware drivers, communication channels, and tasks
# for controlling two motors with encoders. It runs a real-time scheduler
# that coordinates motor control and user interface tasks.


# ============================================================================
# IMPORTS
# ============================================================================

import gc

# Hardware drivers and timing
print("Importing Micropyhthon")
from pyb import Timer, I2C
gc.collect()
print(gc.mem_free())
print("Loading Task User")
from task_user import task_user
gc.collect()
print(gc.mem_free())

print("Loading motor and sensor control")
# Motor and sensor control
from drivers.motor import Motor
from drivers.encoder import Encoder
from drivers.reflectance import Reflectance_Sensor
from drivers.imu import BNO055
gc.collect()
print(gc.mem_free())

print("Loading Constants")
# Configuration constants
from constants import *
gc.collect()
print(gc.mem_free())

print("Loading other tasks")
# Task implementations
from task_motor import task_motor
gc.collect()
from task_line_follow import task_line_follow
from task_reflectance import task_reflectance
from task_imu import task_imu
from task_observer import task_observer
gc.collect()
from task_observer import task_observer
gc.collect()
print(gc.mem_free())

print("Loaing Scheduling")
# Inter-task communication and scheduling
from task_share import Share, Queue, show_all
from cotask import Task, task_list
gc.collect()
print(gc.mem_free())


# ============================================================================
# HARDWARE INITIALIZATION
# ============================================================================

# Configure timers for encoder input capture and motor PWM output
tim1 = Timer(1)             # Right encoder input capture
tim2 = Timer(2)             # Left encoder input capture
tim3 = Timer(3, freq=50000) # Motor PWM frequency (50 kHz)

# Initialize motor driver objects
rightMotor = Motor(MOTOR_R_PWM, tim3, 1, MOTOR_R_SLP, MOTOR_R_DIR, BATT_ADC)
leftMotor = Motor(MOTOR_L_PWM, tim3, 2, MOTOR_L_SLP, MOTOR_L_DIR, BATT_ADC)

# Initialize encoder feedback objects
rightEncoder = Encoder(tim1, ENCODER_R_A, ENCODER_R_B)
leftEncoder = Encoder(tim2, ENCODER_L_A, ENCODER_L_B)

reflectanceSensor = Reflectance_Sensor([
    QTRX_A1,
    QTRX_A3,
    QTRX_A5,
    QTRX_A7,
    QTRX_A9,
    QTRX_A11,
    QTRX_A13
    ])

imuSensor = BNO055(I2C(1, baudrate=100000))



# ============================================================================
# INTER-TASK COMMUNICATION SETUP
# ============================================================================

# Left motor control shares
leftMotorGo        = Share("B", name="Left Mot. Go Flag")
leftMotorSetPoint  = Share("f", name="Left Mot. Set Point")
leftMotorKp        = Share("f", name="Left Mot. Kp Gain")
leftMotorKi        = Share("f", name="Left Mot. Ki Gain")

# Right motor control shares
rightMotorGo       = Share("B", name="Right Mot. Go Flag")
rightMotorSetPoint = Share("f", name="Right Mot. Set Point")
rightMotorKp       = Share("f", name="Right Mot. Kp Gain")
rightMotorKi       = Share("f", name="Right Mot. Ki Gain")

# Data collection buffers (queues for one-way data transmission)
dataValues         = Queue("f", 50, name="Data Collection Buffer")
timeValues         = Queue("L", 50, name="Time Buffer")

# Centroid logging buffers (for line-follow plotting)
centroidValues     = Queue("f", 50, name="Centroid Buffer")
centroidTimeValues = Queue("L", 50, name="Centroid Time Buffer")

# Line following sensor shares
lineCentroid       = Share("f", name="Line Centroid Val")
lineFound          = Share("B", name="Line Found Flag") # For future use
lineFollowGo       = Share("B", name="Line Follow Go Flag")
lineFollowSetPoint = Share("f", name="Line Follow Set Point")
lineFollowKp       = Share("f", name="Line Follow Kp Gain")
lineFollowKi       = Share("f", name="Line Follow Ki Gain")
lineFollowKff      = Share("f", name="Line Follow Kff Gain")

# Load default gains
# Note: tasks will try to read from
# saved configuration on their first run
lineFollowKp.put(DEFAULT_LF_KP)
lineFollowKi.put(DEFAULT_LF_KI)
leftMotorKp.put(DEFAULT_MOTOR_KP)
rightMotorKp.put(DEFAULT_MOTOR_KP)
leftMotorKi.put(DEFAULT_MOTOR_KI)
rightMotorKi.put(DEFAULT_MOTOR_KI)

# Reflectance sensor array shares
#  Mode:
#   - 0: Idle
#   - 1: Calibration Mode (dark)
#   - 2: Calibration Mode (light)
#   - 3: Running
reflectanceMode    = Share("B", name="Reflectance Sensor Go Flag")

# IMU shares
imuMode            = Share("B", name="IMU Mode")
imuCalibration     = Share("B", name="IMU Calibration Values")
imuAx              = Share("f", name="IMU Accel X")
imuAy              = Share("f", name="IMU Accel Y")
imuAz              = Share("f", name="IMU Accel Z")
imuGx              = Share("f", name="IMU Gyro X")
imuGy              = Share("f", name="IMU Gyro Y")
imuHeadingRate         = Share("f", name="IMU Heading Rate [rad/s]")
headingRate        = Share("f", name="IMU Heading Rate")

# Observer raw-input shares (unit normalization will happen in observer task)
motorVoltageLeft    = Share("f", name="Left Motor Voltage [V]")
motorVoltageRight   = Share("f", name="Right Motor Voltage [V]")
wheelDistLeft      = Share("f", name="Wheel Dist Left [mm]")
wheelDistRight     = Share("f", name="Wheel Dist Right [mm]")
imuHeading          = Share("f", name="IMU Heading [rad]")

motorOmegaLeft    = Share("f", name="Left Motor Angular Velocity [rad/s]")
motorOmegaRight   = Share("f", name="Right Motor Angular Velocity [rad/s]")

observerHeading    = Share("f", name="Observer Heading [rad]")
observerHeadingRate    = Share("f", name="Observer Heading Rate [rad/s]")
observerCenterDistance = Share("f", name="Observer Center Distance [mm]")
observerOmegaLeft    = Share("f", name="Observer Left Motor Angular Velocity [rad/s]")
observerOmegaRight    = Share("f", name="Observer Right Motor Angular Velocity [rad/s]")
observerDistanceLeft    = Share("f", name="Observer Left Wheel Distance [mm]")
observerDistanceRight    = Share("f", name="Observer Right Wheel Distance [mm]")

# ============================================================================
# TASK INSTANTIATION
# ============================================================================

# Create motor control task objects with shared communication channels
leftMotorTask = task_motor(
    leftMotor, leftEncoder,
    leftMotorGo, leftMotorKp, leftMotorKi, leftMotorSetPoint,
    dataValues, timeValues, wheelDistLeft, motorVoltageLeft, motorOmegaLeft)

rightMotorTask = task_motor(
    rightMotor, rightEncoder,
    rightMotorGo, rightMotorKp, rightMotorKi, rightMotorSetPoint,
    dataValues, timeValues, wheelDistRight, motorVoltageRight, motorOmegaRight)

# Create a line follower control instance
lineFollowTask = task_line_follow(
    lineFollowGo,
    lineFollowSetPoint,
    lineFollowKp,
    lineFollowKi,
    lineCentroid,
    lineFollowKff,
    rightMotorSetPoint,
    leftMotorSetPoint
)

# Create a reflectance sensor array instance
reflectanceTask = task_reflectance(
    reflectanceSensor,
    reflectanceMode,
    lineCentroid,
    lineFound,
    centroidValues,
    centroidTimeValues
)

# Create an IMU sensor instance
imuTask = task_imu(
    imuSensor, imuMode, imuCalibration,
    imuAx, imuAy, imuAz,
    imuGx, imuGy, imuHeadingRate,
    imuHeading, headingRate
)

# Create an Observer instance
observerTask = task_observer(
    wheelDistLeft,
    wheelDistRight,
    motorVoltageLeft,
    motorVoltageRight,
    imuHeading,
    imuHeadingRate,
    observerCenterDistance,
    observerHeading,
    observerHeadingRate,
    observerOmegaLeft,
    observerOmegaRight,
    observerDistanceLeft,
    observerDistanceRight
)

# Create user interface task for parameter adjustment and data collection
userTask = task_user(
    leftMotorGo, leftMotorKp, leftMotorKi, leftMotorSetPoint,
    rightMotorGo, rightMotorKp, rightMotorKi, rightMotorSetPoint,
    dataValues, timeValues,
    centroidValues, centroidTimeValues,
    reflectanceMode, reflectanceSensor,
    lineFollowGo, lineFollowSetPoint, lineFollowKp,
    lineFollowKi, lineCentroid, lineFollowKff,
    imuMode, imuCalibration,
    imuAx, imuAy, imuAz, imuGx, imuGy, 
    #Shares for State Estimation
    imuHeadingRate, motorVoltageLeft, motorVoltageRight, wheelDistLeft, 
    wheelDistRight, imuHeading, motorOmegaLeft, motorOmegaRight,
    observerCenterDistance, observerHeading, observerHeadingRate, observerOmegaLeft, observerOmegaRight,
    observerDistanceLeft, observerDistanceRight
    )



# ============================================================================
# TASK SCHEDULING SETUP
# ============================================================================

# Register tasks with scheduler
# Priority: 2 (highest) for left motor, 1 for right motor, 0 (lowest) for user interface
# Period: 20 ms (50 Hz) for motor tasks, 0 ms (run as available) for user task
task_list.append(Task(leftMotorTask.run, name="Left Mot. Task",
                      priority=5, period=20, profile=True))
task_list.append(Task(rightMotorTask.run, name="Right Mot. Task",
                      priority=5, period=20, profile=True))
task_list.append(Task(userTask.run, name="User Int. Task",
                      priority=0, period=0, profile=True))
task_list.append(Task(reflectanceTask.run, name="Refl. Sensor Task",
                      priority=4, period=15, profile=True))
task_list.append(Task(lineFollowTask.run, name="Line Follow Task",
                      priority=5, period=15, profile=True))
task_list.append(Task(imuTask.run, name="IMU Task",
                      priority=2, period=20, profile=True))
task_list.append(Task(observerTask.run, name="Observer Task",
                      priority=0, period=20, profile=True))

def garbage_collect():
    while True:
        gc.collect()
        yield 0

task_list.append(Task(garbage_collect, name="Garbage collection",
                      priority=0, period=10000, profile=True))

gc.collect()

# ============================================================================
# MAIN SCHEDULING LOOP
# ============================================================================

# Run the task scheduler continuously until user interrupts with Ctrl-C
while True:
    try:
        # Execute highest-priority ready task
        task_list.pri_sched()

    except KeyboardInterrupt:
        # Graceful shutdown: disable motors and exit
        print("Program Terminating")
        leftMotor.disable()
        rightMotor.disable()
        del userTask
        break

# Print final statistics and status information
print("\n")
print(task_list)
print(show_all())
