"""
task_user.py

User-interface task implemented as a generator-style Task class.
Reads single-character commands from the USB VCP, performs
multi-character inputs when required (via multichar_input), and
updates Shares/Queues to coordinate with motor tasks.

Lightweight, comment-style type hints are included for clarity
and static-readability but are safe for MicroPython (they are comments).
"""
from pyb import USB_VCP, UART, ADC, Pin
from task_share import Share, Queue
import micropython
from multichar_input import multichar_input
from constants import CSV_BEGIN, CSV_END, GAINS_FILE, BATT_ADC
try:
    import ujson as json
except ImportError:
    import json

from pyb import I2C
from drivers.imu import BNO055, NDOF_OP_MODE
import time
import gc
from utime import ticks_us, ticks_diff
from drivers.reflectance import Reflectance_Sensor

# State constants (use micropython.const for efficiency on embedded)
S0_PROMPT = micropython.const(0)        # Initial state: print prompt
S1_CMD = micropython.const(1)           # Wait for single-character command
S2_HELP = micropython.const(2)          # Show help, wait for Enter
S3_GAINS = micropython.const(3)         # Read new Kp / Ki values
S4_SETPOINT = micropython.const(4)      # Read new setpoint
S5_COLLECT = micropython.const(5)       # Wait for data collection to finish
S6_DEBUG = micropython.const(6)         # For Misc Debugging
S7_CALIBRATION = micropython.const(7)   # Calibrate Line Sensor
S8_LINEFOLLOW = micropython.const(8)    # Calibrate Line Sensor
S9_IMU_MENU = micropython.const(9)      # IMU submenu
S10_STATE_ESTIMATION = micropython.const(10) # State Estimation

# Reusable UI text
# UI_prompt = "\r\n\n \
# +-----------------------------------------------------------------------+\n\r \
# | ME 405 Romi Tuning Interface Help Menu                                |\n\r \
# +---+-------------------------------------------------------------------+\n\r \
# | h | Print help menu                                                   |\n\r \
# | k | Enter new gain values                                             |\n\r \
# | s | Choose a new setpoint                                             |\n\r \
# | g | Trigger step response and print results                           |\n\r \
# | c | Line Sensor Calibration                                           |\n\r \
# | l | Follow Line                                                       |\n\r \
# | i | Misc Debug                                                        |\n\r \
# +---+-------------------------------------------------------------------+\n\r \
# \n\r \
# >: "


class task_user:
    # UI task class used by the cooperative scheduler.

    # The `run()` method is a generator that yields the current state each
    # iteration so it can cooperate with cotask scheduler semantics.
    

    def __init__(
        self,
        leftMotorGo,           # type: Share
        leftMotorKp,           # type: Share
        leftMotorKi,           # type: Share
        leftMotorSetPoint,     # type: Share
        rightMotorGo,          # type: Share
        rightMotorKp,          # type: Share
        rightMotorKi,          # type: Share
        rightMotorSetPoint,    # type: Share
        dataValues,            # type: Queue
        timeValues,            # type: Queue
        centroidValues,        # type: Queue
        centroidTimeValues,    # type: Queue
        reflectanceMode,       # type: Share
        reflectanceSensor,     # type: Reflectance_Sensor
        lineFollowGo,          # type: Share
        lineFollowSetPoint,    # type: Share
        lineFollowKp,          # type: Share
        lineFollowKi,          # type: Share
        lineCentroid,          # type: Share
        lineFollowKff,          # type: Share
        imuMode,               # type: Share
        imuCalibration,        # type: Share
        imuAx,                 # type: Share
        imuAy,                 # type: Share
        imuAz,                 # type: Share
        imuGx,                 # type: Share
        imuGy,                 # type: Share
        imuHeadingRate,         # type: Share
        motorVoltageLeft,      # type: Share
        motorVoltageRight,     # type: Share
        wheelDistLeft,         # type: Share
        wheelDistRight,        # type: Share
        imuHeading,            # type: Share
        motorOmegaLeft,        # type: Share
        motorOmegaRight,       # type: Share
        observerCenterDistance,# type: Share
        observerHeading,       # type: Share
        observerHeadingRate,   # type: Share
        observerOmegaLeft,     # type: Share
        observerOmegaRight,    # type: Share
        observerDistanceLeft,  # type: Share
        observerDistanceRight  # type: Share
    ):

        # State machine
        self._state = S0_PROMPT  # current state (int)

        # Shares for left motor (comment hints only)
        self._leftMotorGo = leftMotorGo            # type: Share
        self._leftMotorKp = leftMotorKp            # type: Share
        self._leftMotorKi = leftMotorKi            # type: Share
        self._leftMotorSetPoint = leftMotorSetPoint# type: Share

        # Shares for right motor
        self._rightMotorGo = rightMotorGo          # type: Share
        self._rightMotorKp = rightMotorKp          # type: Share
        self._rightMotorKi = rightMotorKi          # type: Share
        self._rightMotorSetPoint = rightMotorSetPoint  # type: Share

        # Serial interface for host UI over Bluetooth (HC-05)
        # self._ser = UART(3, 38400)               # type: UART
        # Serial interface (USB virtual COM port / REPL-side UI)
        self._ser = USB_VCP()                       # type: USB_VCP

        # Queues used for data collection / logging
        self._dataValues = dataValues               # type: Queue
        self._timeValues = timeValues               # type: Queue
        self._centroidValues = centroidValues       # type: Queue
        self._centroidTimeValues = centroidTimeValues  # type: Queue

        # Shares for reflectance sensor
        self._reflectanceSensor = reflectanceSensor
        self._reflectanceMode = reflectanceMode     # type: Share
        self._lineFollowGo = lineFollowGo           # type: Share
        self._lineFollowSetPoint = lineFollowSetPoint
        self._lineFollowKp = lineFollowKp           # type: Share
        self._lineFollowKi = lineFollowKi           # type: Share
        self._lineCentroid = lineCentroid           # type: Share
        self._lineFollowKff = lineFollowKff         # type: Share

        # Shares for IMU data
        self._imuMode = imuMode
        self._imuCalib = imuCalibration
        self._Ax = imuAx
        self._Ay = imuAy
        self._Az = imuAz
        self._Gx = imuGx
        self._Gy = imuGy
        self._imuHeadingRate = imuHeadingRate

        # Shares for state-estimation comparison
        self._motorVoltageLeft = motorVoltageLeft
        self._motorVoltageRight = motorVoltageRight
        self._wheelDistLeft = wheelDistLeft
        self._wheelDistRight = wheelDistRight
        self._imuHeading = imuHeading
        self._motorOmegaLeft = motorOmegaLeft
        self._motorOmegaRight = motorOmegaRight
        self._observerCenterDistance = observerCenterDistance
        self._observerHeading = observerHeading
        self._observerHeadingRate = observerHeadingRate
        self._observerOmegaLeft = observerOmegaLeft
        self._observerOmegaRight = observerOmegaRight
        self._observerDistanceLeft = observerDistanceLeft
        self._observerDistanceRight = observerDistanceRight

        # Battery adc reading
        # self._battAdc = ADC(Pin(BATT_ADC))

        # Notify user task instantiation (kept from original behavior)
        self._ser.write("User Task object instantiated\r\n")

    def _save_gains(self) -> bool:
        data = {
            "motor": {
                "kp": self._leftMotorKp.get(),
                "ki": self._leftMotorKi.get(),
            },
            "line_follower": {
                "kp": self._lineFollowKp.get(),
                "ki": self._lineFollowKi.get(),
                "set_point": self._lineFollowSetPoint.get(),
                "kff": self._lineFollowKff.get()
            },
        }

        try:
            with open(GAINS_FILE, "w") as gains_file:
                json.dump(data, gains_file)
        except OSError:
            return False

        return True

    def run(self):
        # Generator for the task's behavior.

        # Yields:
        #     current state (int) at the end of each loop iteration so the
        #     cooperative scheduler can manage it.
        
        while True:

            # -----------------------
            # S0_PROMPT: print prompt
            # -----------------------
            if self._state == S0_PROMPT:
                while True:
                    self._ser.write("\r\n\n")
                    self._ser.write("+-----------------------------------------------------------------------+\r\n")
                    self._ser.write("| ME 405 Romi Tuning Interface Help Menu                                |\r\n")
                    self._ser.write("+---+-------------------------------------------------------------------+\r\n")
                    self._ser.write("| h | Print help menu                                                   |\r\n")
                    yield
                    self._ser.write("| k | Enter new gain values                                             |\r\n")
                    self._ser.write("| s | Choose a new setpoint                                             |\r\n")
                    self._ser.write("| g | Trigger step response and print results                           |\r\n")
                    self._ser.write("| c | Line Sensor Calibration                                           |\r\n")
                    yield
                    self._ser.write("| b | BNO055 IMU menu                                                   |\r\n")
                    self._ser.write("| l | Follow Line                                                       |\r\n")
                    self._ser.write("| e | State Estimation                                                  |\r\n")
                    self._ser.write("| i | Misc Debug                                                        |\r\n")
                    self._ser.write("+---+-------------------------------------------------------------------+\r\n")
                    self._ser.write("\r\n")
                    self._ser.write(">: ")

                    break

                self._state = S1_CMD

            # -----------------------
            # S1_CMD: single-char commands
            # -----------------------
            elif self._state == S1_CMD:
                # Only proceed if a character is available
                if self._ser.any():
                    inChar = self._ser.read(1).decode()

                    # Help menu
                    if inChar in {"h", "H"}:
                        self._ser.write(f"{inChar}\r\n")
                        self._state = S2_HELP

                    # Enter gains
                    elif inChar in {"k", "K"}:
                        self._ser.write(f"{inChar}\r\n")
                        self._state = S3_GAINS

                    # Enter setpoint
                    elif inChar in {"s", "S"}:
                        self._ser.write(f"{inChar}\r\n")
                        self._state = S4_SETPOINT

                    # Trigger step response / data collection
                    elif inChar in {"g", "G"}:
                        self._ser.write(f"{inChar}\r\n")
                        # Enable the right motor test (preserved behavior)
                        self._rightMotorGo.put(2) # Set 2 for profiling
                        self._leftMotorGo.put(1)  # Set 1 for just spinning (not collecting data)
                        self._state = S5_COLLECT

                    # Line Sensor Calibration
                    elif inChar in {"c", "C"}:
                        self._ser.write(f"{inChar}\r\n")
                        self._state = S7_CALIBRATION

                    # Debug
                    elif inChar in {"i", "I"}:
                        self._ser.write(f"{inChar}\r\n")
                        self._state = S6_DEBUG

                    # Line Follow
                    elif inChar in {"l", "L"}:
                        self._ser.write(f"{inChar}\r\n")
                        self._state = S8_LINEFOLLOW

                    # State Estimation Compare
                    elif inChar in {"e", "E"}:
                        self._ser.write(f"{inChar}\r\n")
                        self._state = S10_STATE_ESTIMATION

                    elif inChar in {"b", "B"}:
                        self._ser.write(f"{inChar}\r\n")
                        self._state = S9_IMU_MENU

                    elif inChar in {"\r", "\n"}:
                        while self._ser.any():
                            self._ser.read(1)
                        
                        self._state = S0_PROMPT

            # -----------------------
            # S2_HELP: display help, wait for Enter
            # -----------------------
            elif self._state == S2_HELP:
                self._ser.write("Help Menu\r\n")
                self._ser.write("-(Test)\r\n")
                self._ser.write("-(Test)\r\n")
                self._ser.write("Press enter to continue")

                # Wait for newline or carriage return, non-blocking (yielding)
                while True:
                    if self._ser.any():
                        inChar = self._ser.read(1).decode()
                        if inChar in {"\r", "\n"}:
                            # Clear any remaining input then exit help
                            while self._ser.any():
                                self._ser.read(None)
                            break
                    yield
                # Return to main prompt
                self._state = S0_PROMPT

            # -----------------------
            # S3_GAINS: prompt for motor and line following Kp and Ki (uses multichar_input)
            # -----------------------
            elif self._state == S3_GAINS:
                # Prompt and read new motor Kp (assumes same gain for L/R)
                self._ser.write(
                    f"Current motor Kp gain: {self._leftMotorKp.get():.2f}\r\n"
                )
                self._ser.write("Enter a new motor Kp gain value: \r\n->: ")
                value = yield from multichar_input(self._ser)

                if value is not None:
                    if value <= 0:
                        self._ser.write("Invalid motor Kp gain (<= 0). Motor Kp gain unchanged")
                    else:
                        # Apply same Kp to both motors (preserved behavior)
                        self._leftMotorKp.put(value)
                        self._rightMotorKp.put(value)
                        self._ser.write(f"Motor Kp Gain Set To: {value:.2f}\r\n")
                else:
                    self._ser.write("No value entered. Motor Kp gains unchanged.\r\n")

                # Prompt and read new motor Ki (assumes same gain for L/R)
                self._ser.write(
                    f"\r\nCurrent motor Ki gains: {self._leftMotorKi.get():.2f}\r\n"
                )
                self._ser.write("Enter a new motor Ki gain value:\r\n->: ")
                value = yield from multichar_input(self._ser)

                if value is not None:
                    if value < 0:
                        self._ser.write("Invalid motor Ki gain (< 0). Motor Ki gains unchanged")
                    else:
                        self._leftMotorKi.put(value)
                        self._rightMotorKi.put(value)
                        self._ser.write(f"Motor Ki Gain Set To: {value} \r\n")
                else:
                    self._ser.write("No value entered. Motor Ki gains unchanged.\r\n")

                # Prompt and read new line follower Kp
                self._ser.write(
                    f"\r\nCurrent line follower (LF) Kp gain: {self._lineFollowKp.get():.2f}\r\n"
                )
                self._ser.write("Enter a new LF Kp gain value:\r\n->: ")
                value = yield from multichar_input(self._ser)

                if value is not None:
                    if value < 0:
                        self._ser.write("Invalid LF Kp gain (< 0). LF Kp gains unchanged")
                    else:
                        self._lineFollowKp.put(value)
                        self._ser.write(f"LF Kp Gain Set To: {value}\r\n")
                else:
                    self._ser.write("No value entered. LF Kp gains unchanged.\r\n")

                # Prompt and read new line follower Ki
                self._ser.write(
                    f"\r\nCurrent line follower (LF) Ki gain: {self._lineFollowKi.get():.2f}\r\n"
                )
                self._ser.write("Enter a new LF Ki gain value:\r\n->: ")
                value = yield from multichar_input(self._ser)

                if value is not None:
                    if value < 0:
                        self._ser.write("Invalid LF Ki gain (< 0). LF Ki gains unchanged")
                    else:
                        self._lineFollowKi.put(value)
                        self._ser.write(f"LF Ki Gain Set To: {value}\r\n")
                else:
                    self._ser.write("No value entered. LF Ki gains unchanged.\r\n")

                # Prompt and read new LF feed forward gain
                self._ser.write(
                    f"\r\nCurrent LF Kff gain: {self._lineFollowKff.get():.2f}\r\n"
                )
                self._ser.write("Enter a new LF Kff gain value:\r\n->: ")
                value = yield from multichar_input(self._ser)

                if value is not None:
                    if value < 0:
                        self._ser.write("Invalid LF Kff gain (< 0). LF Kff gain unchanged")
                    else:
                        self._lineFollowKff.put(value)
                        self._ser.write(f"LF Kff Gain Set To: {value}\r\n")
                else:
                    self._ser.write("No value entered. LF Kff gain unchanged.\r\n")

                # Show final values and return to prompt
                self._ser.write("\r\nController gains:\r\n")
                self._ser.write(f" - Motor Kp: {self._leftMotorKp.get():.2f}\r\n")
                self._ser.write(f" - Motor Ki: {self._leftMotorKi.get():.2f}\r\n")
                self._ser.write(f" - Line follower Kp: {self._lineFollowKp.get():.2f}\r\n")
                self._ser.write(f" - Line follower Ki: {self._lineFollowKi.get():.2f}\r\n")
                self._ser.write(f" - Line follower Kff: {self._lineFollowKff.get():.2f}\r\n")
                if not self._save_gains():
                    self._ser.write(f"Warning: failed to save {GAINS_FILE}.\r\n")

                self._state = S0_PROMPT

            # -----------------------
            # S4_SETPOINT: prompt for setpoints (uses multichar_input)
            # -----------------------
            elif self._state == S4_SETPOINT:
                self._ser.write(f"\r\nCurrent motor setpoint: {self._leftMotorSetPoint.get():.2f}\r\n")
                self._ser.write("\r\nEnter new motor setpoint: \r\n->: ")
                value = yield from multichar_input(self._ser)

                if value is not None:
                    self._leftMotorSetPoint.put(value)
                    self._rightMotorSetPoint.put(value)
                    self._ser.write(f"\r\nSetpoint Set To: {value:.2f}")
                else:
                    self._ser.write("\r\nNo setpoint specified. Value unchanged.")


                self._ser.write(f"\r\nCurrent line follow setpoint: {self._lineFollowSetPoint.get():.2f}\r\n")
                self._ser.write("\r\nEnter new LF setpoint: \r\n->: ")
                value = yield from multichar_input(self._ser)

                if value is not None:
                    self._lineFollowSetPoint.put(value)
                    self._ser.write(f"\r\nLF Setpoint Set To: {value:.2f}")
                else:
                    self._ser.write("\r\nNo LF setpoint specified. Value unchanged.")

                self._save_gains()

                self._state = S0_PROMPT

            # -----------------------
            # S5_COLLECT: wait while data collection runs
            # -----------------------
            elif self._state == S5_COLLECT:
                # Discard any incoming characters while data is collecting
                if self._ser.any():
                    self._ser.read(1)

                # When queues are full, stop the motor and print results
                if self._dataValues.full() or self._timeValues.full():
                    # Disable motors
                    self._leftMotorGo.put(0)
                    self._rightMotorGo.put(0)

                    yield # Let motors pick up the new command

                    self._ser.write("Data collection complete...\r\n")
                    self._ser.write(f"{CSV_BEGIN}\r\n")
                    self._ser.write("Time (ms), Velocity (mm/s)\r\n")

                    while True:
                        if self._dataValues.any():
                            # Print one sample (time, data) per iteration
                            self._ser.write(
                                f"{self._timeValues.get()},{self._dataValues.get()}\r\n"
                            )
                        else:
                            self._ser.write(f"{CSV_END}\r\n")
                            self._state = S0_PROMPT
                            break

                        yield


            # -----------------------
            # S6_DEBUG: debug state
            # -----------------------
            elif self._state == S6_DEBUG:
                self._ser.write("DEBUG\r\n")

                ############################################
                # LINE SENSOR CENTROID VISUALIZATION
                ############################################
                raw, calibrated, value = self._reflectanceSensor.get_values()
                self._ser.write(f" RAW   CALIBRATED  \r\n")
                for i in range(len(raw)):
                    self._ser.write(f"{raw[i]:04d}")
                    self._ser.write('   ')
                    for _ in range(int(calibrated[i]*10)):
                        self._ser.write('+')
                    self._ser.write("\r\n")

                self._ser.write(f"Measured centroid: {value:.2f}\r\n\n")
                # self._ser.write("\r\nPlease Enter a Speed: \r\n->: ")
                # value = yield from multichar_input(self._ser)
                ############################################

                ############################################
                # # BATTERY DROOP COMPENSATION
                ############################################
                # adcVal = self._battAdc.read()

                # adcVoltage = (adcVal / 4096 * 3.3)

                # # Scale for 4.7k and 10k voltage divider
                # battVoltage = adcVoltage / 0.305

                # # Calculate scaling factor
                # effortScale = 6.5 / battVoltage

                # self._ser.write(f'Voltage: {battVoltage}V, Scale: {effortScale}, output: {battVoltage * effortScale}\r\n')
                ############################################


                ############################################
                # BATTERY LEVEL
                ############################################
                # --- Scale value to account for battery droop
                adcVoltage = ADC(BATT_ADC).read() / 4096 * 3.3

                # Scale for 4.7k and 10k voltage divider.
                # (Slightly tweaked to account for actual VDD)
                battVoltage = adcVoltage / 0.305

                self._ser.write(f"Battery voltage: {battVoltage:.2f}V\r\n")


                ############################################
                # IMU
                ############################################
                # yield from self._imu.tare_accel_gyro()

                # # Give imu time to settl
                # time.sleep_ms(10)
                # yield
                # time.sleep_ms(10)
                # yield
                # time.sleep_ms(10)
                # yield

                # ax = self._Ax.get()
                # ay = self._Ay.get()
                # az = self._Az.get()
                # gx = self._Gx.get()
                # gy = self._Gy.get()
                # gz = self._Gz.get()

                # self._ser.write("Accelerometer data: \r\n")
                # for x in (ax, ay, az, gx, gy, gz):
                #     self._ser.write(f"  {x}\r\n")

                # self._ser.write("\r\n")



                # Return to main prompt
                self._state = S0_PROMPT

            # -----------------------
            # S7_CALIBRATE: calibration state
            # -----------------------
            elif self._state == S7_CALIBRATION:
                self._ser.write("CALIBRATION\r\n")

                # Double check this is the action the user wants to make
                self._ser.write("Would you like to begin calibration (y/n)?\r\n")

                # Check for a valid keystroke
                while True:
                    if self._ser.any():
                        inChar = self._ser.read(1).decode()
                        if inChar in {"y", "Y", "n", "N"}:
                            break

                # If NO selected, return to main menu from this point
                if inChar in {"n", "N"}:
                    self._state = S0_PROMPT

                    yield 0
                    continue

                # Begin with light side calibration
                self._ser.write("Place Romi on a light surface. Press Enter when ready\r\n")
                # Wait for ENT (cooperative)
                while True:
                    if self._ser.any():
                        inChar = self._ser.read(1).decode()
                        if inChar in {"\r", "\n"}:
                            # Clear any remaining input then exit help
                            while self._ser.any():
                                self._ser.read(None)
                            break
                    yield 0

                # Request that reflectance sensor goes into light calib. mode
                self._reflectanceMode.put(2)

                # Wait for 'ack' that calibration is done (mode = 0)
                while self._reflectanceMode.get() != 0:
                    yield 0

                # Then perform dark side calibration
                self._ser.write("Please Place Romi On a Dark Surface. Press Enter When Complete\r\n")

                # Wait for ENT (cooperative)
                while True:
                    if self._ser.any():
                        inChar = self._ser.read(1).decode()
                        if inChar in {"\r", "\n"}:
                            # Clear any remaining input then exit help
                            while self._ser.any():
                                self._ser.read(None)
                            break
                    yield 0

                # Request that reflectance sensor goes into light calib. mode
                self._reflectanceMode.put(1)

                # Wait for 'ack' that calibration is done (mode = 0)
                while self._reflectanceMode.get() != 0:
                    yield 0

                self._ser.write('Sensor calibration complete.\r\n')

                # Return to main prompt
                self._state = S0_PROMPT

            # -----------------------
            # S8_LINEFOLLOW: Line Following State
            # -----------------------
            elif self._state == S8_LINEFOLLOW:
                self._ser.write("Line Follow Mode\r\n")
                # self._UART.write("Line Follow Mode\r\n")

                # Set sensor array into RUN mode
                self._reflectanceMode.put(3)
                self._leftMotorGo.put(1)
                self._rightMotorGo.put(1)
                yield # Let sensor array pick up the change

                # Enable line following controller
                self._lineFollowGo.put(1)

                self._ser.write("Line Following Started. Please Press Enter to Stop.\r\n")

                # Wait for newline or carriage return, non-blocking (yielding)
                self._ser.write(f"{CSV_BEGIN}\r\n")
                stream_decimation = 5
                sample_idx = 0
                while True:
                    # Decimate queued samples; stream only every Nth row to reduce CSV volume.
                    """if self._centroidTimeValues.any() and self._centroidValues.any():
                        t_ms = self._centroidTimeValues.get()
                        centroid = self._centroidValues.get()
                        if (sample_idx % stream_decimation) == 0:
                            self._ser.write(f"{t_ms},{centroid}\r\n")
                        sample_idx += 1"""

                    if self._ser.any():
                        inChar = self._ser.read(1).decode()
                        if inChar in {"\r", "\n"}:
                            # Clear any remaining input then exit help
                            while self._ser.any():
                                self._ser.read(None)
                            break
                    yield 0
                    
                # Stop line-following mode and related tasks before returning.
                self._ser.write(f"{CSV_END}\r\n")

                self._lineFollowGo.put(0)
                self._reflectanceMode.put(0)
                self._leftMotorGo.put(0)
                self._rightMotorGo.put(0)
                self._leftMotorSetPoint.put(0)
                self._rightMotorSetPoint.put(0)

                
                # Return to main prompt
                self._state = S0_PROMPT

            # -----------------------
            # S9_IMU_Menue: Line Following State
            # -----------------------
            elif self._state == S9_IMU_MENU:
                self._ser.write("\r\n")
                self._ser.write("  +--------------------------------+\r\n")
                self._ser.write("  | IMU Submenu                    |\r\n")
                self._ser.write("  +--------------------------------+\r\n")
                self._ser.write("  | 1: Load calibration            |\r\n")
                self._ser.write("  | 2: Save calibration            |\r\n")
                self._ser.write("  | 3: Tare accel and gyro         |\r\n")
                self._ser.write("  | 4: Read and print values       |\r\n")
                self._ser.write("  | 5: Get calibration states      |\r\n")
                self._ser.write("  |                                |\r\n")
                self._ser.write("  | 0: Exit                        |\r\n")
                self._ser.write("  +--------------------------------+\r\n")
                self._ser.write("  ->: ")

                while True:
                    if self._ser.any():
                        inChar = self._ser.read(1).decode('ascii')

                        if inChar.isdigit():
                            user_sel = int(inChar)

                            # Load calibration from file
                            if user_sel == 0:
                                self._ser.write(f"{inChar}\r\n")

                                self._state = S0_PROMPT
                                break # Return to main menu

                            elif user_sel == 1:
                                self._ser.write(f"{inChar}\r\n")
                                self._imuMode.put(1)

                                while True:
                                    if self._imuMode.get() == 0:
                                        break
                                    yield
                                self._ser.write("Calibration loaded.\r\n")

                                break # Redraw imu submenu

                            # Save calibration to file
                            elif user_sel == 2:
                                self._ser.write(f"{inChar}\r\n")
                                self._imuMode.put(2)
                                
                                while True:
                                    if self._imuMode.get() == 0:
                                        break
                                    yield
                                self._ser.write("Calibration saved.\r\n")

                                break # Redraw imu submenu

                            # Tare readings
                            elif user_sel == 3:
                                self._ser.write(f"{inChar}\r\n")
                                self._imuMode.put(3)
                                
                                while True:
                                    if self._imuMode.get() == 0:
                                        break
                                    yield

                                self._ser.write("  Readings tared.\r\n")

                                break # Redraw imu submenu

                            # Read and print
                            elif user_sel == 4:
                                self._ser.write(f"{inChar}\r\n")
                                self._imuMode.put(4)
                                while True:
                                    if self._imuMode.get() == 0:
                                        ax = self._Ax.get()
                                        ay = self._Ay.get()
                                        az = self._Az.get()
                                        gx = self._Gx.get()
                                        gy = self._Gy.get()
                                        gz = self._imuHeadingRate.get()

                                        self._ser.write("  Accelerometer data: \r\n")
                                        self._ser.write(f"   - x: {ax}\r\n")
                                        self._ser.write(f"   - y: {ay}\r\n")
                                        self._ser.write(f"   - z: {az}\r\n")
                                        self._ser.write("  Gyroscope data: \r\n")
                                        self._ser.write(f"   - x: {gx}\r\n")
                                        self._ser.write(f"   - y: {gy}\r\n")
                                        self._ser.write(f"   - z: {gz}\r\n")
                                        break
                                    yield

                                break # Redraw imu submenu

                            # Get calibration status
                            elif user_sel == 5:
                                self._ser.write(f"{inChar}\r\n")
                                self._imuMode.put(5)
                                while True:
                                    if self._imuMode.get() == 0:
                                        raw_calib = self._imuCalib.get()

                                        s_sys = (raw_calib >> 6) & 0x03
                                        s_gyro = (raw_calib >> 4) & 0x03
                                        s_accel = (raw_calib >> 2) & 0x03
                                        s_mag = raw_calib & 0x03

                                        self._ser.write("  Calibration status: \r\n")
                                        self._ser.write(f"   - System: {s_sys}\r\n")
                                        self._ser.write(f"   - Gyro:   {s_gyro}\r\n")
                                        self._ser.write(f"   - Accel:  {s_accel}\r\n")
                                        self._ser.write(f"   - Mag:    {s_mag}\r\n")

                                        break
                                    yield
                                    
                                break # Redraw imu submenu


                    yield

            #Need to import a logging thing which we can then graph.
            

            # -----------------------
            # S10_State_Estimation: State Estimation Test
            # -----------------------                    
            elif self._state == S10_STATE_ESTIMATION:
                self._ser.write("State Estimation Test\r\n")

                # Set sensor array into RUN mode
                self._reflectanceMode.put(3)
                self._leftMotorGo.put(1)
                self._rightMotorGo.put(1)
                yield # Let sensor array pick up the change

                # Enable line following controller
                self._lineFollowGo.put(1)
                # Run IMU continuously while in this mode
                self._imuMode.put(0xFF)

                self._ser.write("Line Following Started. Please Press Enter to Stop.\r\n")

                # Wait for newline or carriage return, non-blocking (yielding)
                self._ser.write(f"{CSV_BEGIN}\r\n")
                self._ser.write(
                    "Time (ms),"
                    "centerDistance_sensor (mm),centerDistance_observer (mm),centerDistance_error (mm),"
                    "distL_sensor (mm),distL_observer (mm),distL_error (mm),"
                    "distR_sensor (mm),distR_observer (mm),distR_error (mm),"
                    "heading_sensor,heading_observer,heading_error,"
                    "headingRate_sensor,headingRate_observer,headingRate_error,"
                    "omegaL_sensor (rad/s),omegaL_observer (rad/s),omegaL_error (rad/s),"
                    "omegaR_sensor (rad/s),omegaR_observer (rad/s),omegaR_error (rad/s)\r\n"
                )

                stream_decimation = 5
                sample_idx = 0
                start_time = ticks_us()

                while True:
                    if (sample_idx % stream_decimation) == 0:
                        t_ms = int(ticks_diff(ticks_us(), start_time) / 1000)
                        wheelDistLeft = self._wheelDistLeft.get()
                        wheelDistRight = self._wheelDistRight.get()
                        observerDistanceLeft = self._observerDistanceLeft.get()
                        observerDistanceRight = self._observerDistanceRight.get()
                        imuHeading = self._imuHeading.get()
                        observerHeading = self._observerHeading.get()
                        imuHeadingRate = self._imuHeadingRate.get()
                        observerHeadingRate = self._observerHeadingRate.get()
                        motorOmegaLeft = self._motorOmegaLeft.get()
                        observerOmegaLeft = self._observerOmegaLeft.get()
                        motorOmegaRight = self._motorOmegaRight.get()
                        observerOmegaRight = self._observerOmegaRight.get()
                        centerDistance_sensor = 0.5 * (wheelDistLeft + wheelDistRight)
                        centerDistance_observer = self._observerCenterDistance.get()

                        # Error calculations (sensor - observer)
                        centerDistance_error = centerDistance_sensor - centerDistance_observer
                        distL_error = wheelDistLeft - observerDistanceLeft
                        distR_error = wheelDistRight - observerDistanceRight
                        heading_error = imuHeading - observerHeading
                        headingRate_error = imuHeadingRate - observerHeadingRate
                        omegaL_error = motorOmegaLeft - observerOmegaLeft
                        omegaR_error = motorOmegaRight - observerOmegaRight
                        
                        """
                        line = (
                            "{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{}\r\n".format(
                                t_ms,
                                centerDistance_sensor, centerDistance_observer, centerDistance_error,
                                wheelDistLeft, observerDistanceLeft, distL_error,
                                wheelDistRight, observerDistanceRight, distR_error,
                                imuHeading, observerHeading, heading_error,
                                imuHeadingRate, observerHeadingRate, headingRate_error,
                                motorOmegaLeft, observerOmegaLeft, omegaL_error,
                                motorOmegaRight, observerOmegaRight, omegaR_error,
                            )
                        )
                        self._ser.write(line)"""
                        error_line = (
                            "t_ms={}, centerDistance_error={}, distL_error={}, distR_error={}, "
                            "heading_error={}, headingRate_error={}, omegaL_error={}, omegaR_error={}\r\n".format(
                                t_ms,
                                centerDistance_error,
                                distL_error,
                                distR_error,
                                heading_error,
                                headingRate_error,
                                omegaL_error,
                                omegaR_error,
                            )
                        )
                        self._ser.write(error_line)
                    sample_idx += 1

                    if self._ser.any():
                        inChar = self._ser.read(1).decode()
                        if inChar in {"\r", "\n"}:
                            # Clear any remaining input then exit help
                            while self._ser.any():
                                self._ser.read(None)
                            break
                    yield 0
                    
                # Stop line-following mode and related tasks before returning.
                self._ser.write(f"{CSV_END}\r\n")

                self._lineFollowGo.put(0)
                self._reflectanceMode.put(0)
                self._imuMode.put(0)
                self._leftMotorGo.put(0)
                self._rightMotorGo.put(0)
                self._leftMotorSetPoint.put(0)
                self._rightMotorSetPoint.put(0)

                
                # Return to main prompt
                self._state = S0_PROMPT




            # Yield the current state per scheduler convention
            yield self._state
