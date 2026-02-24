"""
Main control loop for dual-motor closed-loop control system.

This module initializes hardware drivers, communication channels, and tasks
for controlling two motors with encoders. It runs a real-time scheduler
that coordinates motor control and user interface tasks.
"""

# ============================================================================
# IMPORTS
# ============================================================================

import gc

# Hardware drivers and timing
from pyb import Timer
gc.collect()

# Motor and sensor control
from drivers.motor import Motor
gc.collect()
from drivers.encoder import Encoder
gc.collect()
from drivers.reflectance import Reflectance_Sensor
gc.collect()

# Configuration constants
from constants import *
gc.collect()

# Task implementations
from task_motor import task_motor
gc.collect()
from task_user import task_user
gc.collect()
from task_line_follow import task_line_follow
gc.collect()
from task_reflectance import task_reflectance
gc.collect()

# Inter-task communication and scheduling
from task_share import Share, Queue, show_all
gc.collect()
from cotask import Task, task_list
gc.collect()



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
dataValues = Queue("f", 50, name="Data Collection Buffer")
timeValues = Queue("L", 50, name="Time Buffer")

# Centroid logging buffers (for line-follow plotting)
centroidValues = Queue("f", 500, name="Centroid Buffer")
centroidTimeValues = Queue("L", 500, name="Centroid Time Buffer")

# Line following sensor shares
lineCentroid        = Share("f", name="Line Centroid Val")
lineFound           = Share("B", name="Line Found Flag") # For future use
lineFollowGo        = Share("B", name="Line Follow Go Flag")
lineFollowKp        = Share("f", name="Line Follow Kp Gain")
lineFollowKi        = Share("f", name="Line Follow Ki Gain")

# Reflectance sensor array shares
#  Mode:
#   - 0: Idle
#   - 1: Calibration Mode
#   - 2: Running
reflectanceMode      = Share("B", name="Reflectance Sensor Go Flag")


# ============================================================================
# TASK INSTANTIATION
# ============================================================================

# Create motor control task objects with shared communication channels
leftMotorTask = task_motor(
    leftMotor, leftEncoder,
    leftMotorGo, leftMotorKp, leftMotorKi, leftMotorSetPoint,
    dataValues, timeValues)

rightMotorTask = task_motor(
    rightMotor, rightEncoder,
    rightMotorGo, rightMotorKp, rightMotorKi, rightMotorSetPoint,
    dataValues, timeValues)

# Create user interface task for parameter adjustment and data collection
userTask = task_user(
    leftMotorGo, leftMotorKp, leftMotorKi, leftMotorSetPoint,
    rightMotorGo, rightMotorKp, rightMotorKi, rightMotorSetPoint,
    dataValues, timeValues,
    centroidValues, centroidTimeValues,
    reflectanceMode,
    lineFollowGo, lineFollowKp, lineFollowKi, lineCentroid
    )

# Create a line follower control instance
lineFollowTask = task_line_follow(
    lineFollowGo,
    lineFollowKp,
    lineFollowKi,
    lineCentroid,
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


# ============================================================================
# TASK SCHEDULING SETUP
# ============================================================================

# Register tasks with scheduler
# Priority: 2 (highest) for left motor, 1 for right motor, 0 (lowest) for user interface
# Period: 20 ms (50 Hz) for motor tasks, 0 ms (run as available) for user task
task_list.append(Task(leftMotorTask.run, name="Left Mot. Task",
                      priority=2, period=25, profile=True))
task_list.append(Task(rightMotorTask.run, name="Right Mot. Task",
                      priority=1, period=25, profile=True))
task_list.append(Task(userTask.run, name="User Int. Task",
                      priority=0, period=0, profile=False))
task_list.append(Task(reflectanceTask.run, name="Refl. Sensor Task",
                      priority=4, period=25, profile=True))
task_list.append(Task(lineFollowTask.run, name="Line Follow Task",
                      priority=5, period=10, profile=True))

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
        break

# Print final statistics and status information
print("\n")
print(task_list)
print(show_all())
