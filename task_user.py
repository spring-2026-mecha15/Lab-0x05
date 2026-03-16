"""
task_user.py

User-interface task implemented as a generator-style Task class.
Reads single-character commands from the USB VCP, performs
multi-character inputs when required (via multichar_input), and
updates Shares/Queues to coordinate with motor tasks.

Lightweight, comment-style type hints are included for clarity
and static-readability but are safe for MicroPython (they are comments).
"""
from pyb import USB_VCP
import micropython
import sys
import gc
from constants import CSV_BEGIN, CSV_END

sys.path.append('ui')

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
        lineFollowKff,         # type: Share
        imuMode,               # type: Share
        imuCalibration,        # type: Share
        imuHeadingRate,        # type: Share
        motorVoltageLeft,      # type: Share
        motorVoltageRight,     # type: Share
        wheelDistLeft,         # type: Share
        wheelDistRight,        # type: Share
        imuHeading,            # type: Share
        motorOmegaLeft,        # type: Share
        motorOmegaRight,       # type: Share
        observerGoFlag,        # type: Share
        observerCenterDistance,# type: Share
        observerHeading,       # type: Share
        observerHeadingRate,   # type: Share
        observerOmegaLeft,     # type: Share
        observerOmegaRight,    # type: Share
        observerDistanceLeft,  # type: Share
        observerDistanceRight, # type: Share
        competitionGoFlag,     # type: Share
        ultrasonicDistance,    # type: Share
        memMonitorGo,          # type: Share
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
        self._imuHeadingRate = imuHeadingRate

        # Shares for state-estimation comparison
        self._motorVoltageLeft = motorVoltageLeft
        self._motorVoltageRight = motorVoltageRight
        self._wheelDistLeft = wheelDistLeft
        self._wheelDistRight = wheelDistRight
        self._imuHeading = imuHeading
        self._motorOmegaLeft = motorOmegaLeft
        self._motorOmegaRight = motorOmegaRight
        self._observerGoFlag = observerGoFlag
        self._observerCenterDistance = observerCenterDistance
        self._observerHeading = observerHeading
        self._observerHeadingRate = observerHeadingRate
        self._observerOmegaLeft = observerOmegaLeft
        self._observerOmegaRight = observerOmegaRight
        self._observerDistanceLeft = observerDistanceLeft
        self._observerDistanceRight = observerDistanceRight

        self._competitionGoFlag = competitionGoFlag

        self._ultrasonicDistance = ultrasonicDistance

        self._memMonitorGo = memMonitorGo

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
                    self._ser.write("| m | Toggle memory monitor                                             |\r\n")
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

                    # Toggle memory monitor
                    elif inChar in {"m", "M"}:
                        self._ser.write(f"{inChar}\r\n")
                        new_state = 0 if self._memMonitorGo.get() else 1
                        self._memMonitorGo.put(new_state)
                        status = "ON" if new_state else "OFF"
                        self._ser.write(f"Memory monitor: {status}\r\n")

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
                                self._ser.read(1)
                            break
                    yield
                # Return to main prompt
                self._state = S0_PROMPT

            # -----------------------
            # S3_GAINS: prompt for motor and line following Kp and Ki (uses multichar_input)
            # -----------------------
            elif self._state == S3_GAINS:
                gc.collect()
                import ui_gains
                yield from ui_gains.run(self)
                del sys.modules['ui_gains']
                del ui_gains
                gc.collect()
                self._state = S0_PROMPT

            # -----------------------
            # S4_SETPOINT: prompt for setpoints (uses multichar_input)
            # -----------------------
            elif self._state == S4_SETPOINT:
                gc.collect()
                import ui_setpoint
                yield from ui_setpoint.run(self)
                del sys.modules['ui_setpoint']
                del ui_setpoint
                gc.collect()
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
                gc.collect()
                import ui_debug
                yield from ui_debug.run(self)
                del sys.modules['ui_debug']
                del ui_debug
                gc.collect()
                self._state = S0_PROMPT

            # -----------------------
            # S7_CALIBRATE: calibration state
            # -----------------------
            elif self._state == S7_CALIBRATION:
                gc.collect()
                import ui_calibration
                yield from ui_calibration.run(self)
                del sys.modules['ui_calibration']
                del ui_calibration
                gc.collect()
                self._state = S0_PROMPT

            # -----------------------
            # S8_LINEFOLLOW: Line Following State
            # -----------------------
            elif self._state == S8_LINEFOLLOW:
                gc.collect()
                import ui_linefollow
                yield from ui_linefollow.run(self)
                del sys.modules['ui_linefollow']
                del ui_linefollow
                gc.collect()
                self._state = S0_PROMPT

            # -----------------------
            # S9_IMU_Menue: Line Following State
            # -----------------------
            elif self._state == S9_IMU_MENU:
                gc.collect()
                import ui_imu
                yield from ui_imu.run(self)
                del sys.modules['ui_imu']
                del ui_imu
                gc.collect()
                self._state = S0_PROMPT



            # -----------------------
            # S10_State_Estimation: State Estimation Test
            # -----------------------
            elif self._state == S10_STATE_ESTIMATION:
                gc.collect()
                import ui_competition
                yield from ui_competition.run(self)
                del sys.modules['ui_competition']
                del ui_competition
                gc.collect()
                self._state = S0_PROMPT




            # Yield the current state per scheduler convention
            yield self._state

gc.collect()