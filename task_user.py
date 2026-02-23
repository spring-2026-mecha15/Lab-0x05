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
from constants import CSV_BEGIN, CSV_END, GAINS_FILE
try:
    import ujson as json
except ImportError:
    import json

# State constants (use micropython.const for efficiency on embedded)
S0_INIT = micropython.const(0)          # Initial state: print prompt
S1_CMD = micropython.const(1)           # Wait for single-character command
S2_HELP = micropython.const(2)          # Show help, wait for Enter
S3_GAINS = micropython.const(3)         # Read new Kp / Ki values
S4_SETPOINT = micropython.const(4)      # Read new setpoint
S5_COLLECT = micropython.const(5)       # Wait for data collection to finish
S6_DISPLAY_DATA = micropython.const(6)  # Print collected CSV data
S7_DEBUG = micropython.const(7)         # For Misc Debugging
S8_CALIBRATION = micropython.const(8)   # Calibrate Line Sensor
S9_LINEFOLLOW = micropython.const(9)   # Calibrate Line Sensor

# Reusable UI text
UI_prompt = """\r\n\n
+-----------------------------------------------------------------------+\r
| ME 405 Romi Tuning Interface Help Menu                                |\r
+---+-------------------------------------------------------------------+\r
| h | Print help menu                                                   |\r
| k | Enter new gain values                                             |\r
| s | Choose a new setpoint                                             |\r
| g | Trigger step response and print results                           |\r
| c | Line Sensor Calibration                                           |\r
| l | Follow Line                                                       |\r
| i | Misc Debug                                                        |\r
+---+-------------------------------------------------------------------+\r
\r
>: """


class task_user:
    """
    UI task class used by the cooperative scheduler.

    The `run()` method is a generator that yields the current state each
    iteration so it can cooperate with cotask scheduler semantics.
    """

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
        reflectanceMode,       # type: Share
        lineFollowGo,          # type: Share
        lineFollowKp,          # type: Share
        lineFollowKi,          # type: Share
        lineCentroid           # type: Share
    ):
        """
        Initialize the UI task.

        Args (comment-style hints):
            leftMotorGo, rightMotorGo: Share("B") boolean flags for data collection
            leftMotorKp, leftMotorKi, rightMotorKp, rightMotorKi: Share("f") gains
            leftMotorSetPoint, rightMotorSetPoint: Share("f") setpoint values
            dataValues: Queue("f", ...) encoder/sample data
            timeValues: Queue("L", ...) timestamps for collected samples
        """

        # State machine
        self._state = S0_INIT  # current state (int)

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

        # Serial interface (USB virtual COM port)
        self._ser = USB_VCP()                       # type: USB_VCP
        # Serial interface (UART over bluetooth HC-05)
        # self._ser = UART(3, 115200)

        # Queues used for data collection / logging
        self._dataValues = dataValues               # type: Queue
        self._timeValues = timeValues               # type: Queue

        # Shares for reflectance sensor
        self._reflectanceMode = reflectanceMode     # type: Share
        self._lineFollowGo = lineFollowGo           # type: Share
        self._lineFollowKp = lineFollowKp           # type: Share
        self._lineFollowKi = lineFollowKi           # type: Share
        self._lineCentroid = lineCentroid           # type: Share

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
            },
        }

        try:
            with open(GAINS_FILE, "w") as gains_file:
                json.dump(data, gains_file)
        except OSError:
            return False

        return True

    def run(self):
        """
        Generator for the task's behavior.

        Yields:
            current state (int) at the end of each loop iteration so the
            cooperative scheduler can manage it.
        """
        while True:

            # -----------------------
            # S0_INIT: print prompt
            # -----------------------
            if self._state == S0_INIT:
                self._ser.write(UI_prompt)
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
                        self._rightMotorGo.put(1)
                        self._state = S5_COLLECT

                    # Line Sensor Calibration
                    elif inChar in {"c", "C"}:
                        self._ser.write(f"{inChar}\r\n")
                        self._state = S8_CALIBRATION

                    # Debug
                    elif inChar in {"i", "I"}:
                        self._ser.write(f"{inChar}\r\n")
                        self._state = S7_DEBUG

                    # Line Follow
                    elif inChar in {"l", "L"}:
                        self._ser.write(f"{inChar}\r\n")
                        self._state = S9_LINEFOLLOW

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
                self._ser.write(UI_prompt)
                self._state = S1_CMD

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

                # Show final values and return to prompt
                self._ser.write("\r\nController gains:\r\n")
                self._ser.write(f" - Motor Kp: {self._leftMotorKp.get():.2f}\r\n")
                self._ser.write(f" - Motor Ki: {self._leftMotorKi.get():.2f}\r\n")
                self._ser.write(f" - Line follower Kp: {self._lineFollowKp.get():.2f}\r\n")
                self._ser.write(f" - Line follower Ki: {self._lineFollowKi.get():.2f}\r\n")
                if not self._save_gains():
                    self._ser.write(f"Warning: failed to save {_GAINS_FILE}.\r\n")
                self._ser.write(UI_prompt)
                self._state = S1_CMD

            # -----------------------
            # S4_SETPOINT: prompt for setpoint (uses multichar_input)
            # -----------------------
            elif self._state == S4_SETPOINT:
                self._ser.write(f"\r\nCurrent setpoint: {self._leftMotorSetPoint.get():.2f}\r\n")
                self._ser.write("\r\nEnter new setpoint: \r\n->: ")
                value = yield from multichar_input(self._ser)

                if value is not None:
                    self._leftMotorSetPoint.put(value)
                    self._rightMotorSetPoint.put(value)
                    self._ser.write(f"\r\nSetpoint Set To: {value:.2f}")
                else:
                    self._ser.write("\r\nNo setpoint specified. Value unchanged.")

                self._ser.write(UI_prompt)
                self._state = S1_CMD

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

                    self._ser.write("Data collection complete...\r\n")
                    self._ser.write(f"{CSV_BEGIN}\r\n")
                    self._ser.write("Time (ms), Velocity (mm/s)\r\n")
                    self._state = S6_DISPLAY_DATA

            # -----------------------
            # S6_DISPLAY_DATA: print CSV rows from queues
            # -----------------------
            elif self._state == S6_DISPLAY_DATA:
                if self._dataValues.any():
                    # Print one sample (time, data) per iteration
                    self._ser.write(
                        f"{self._timeValues.get()},{self._dataValues.get()}\r\n"
                    )
                else:
                    self._ser.write(f"{CSV_END}\r\n")
                    self._ser.write(UI_prompt)
                    self._state = S1_CMD


            # -----------------------
            # S7_DEBUG: debug state
            # -----------------------
            elif self._state == S7_DEBUG:
                self._ser.write("DEBUG\r\n")

                # LINE SENSOR CENTROID VISUALIZATION
                """
                raw, calibrated, value = self._reflectanceSensor.get_values()
                self._ser.write(f" RAW   CALIBRATED  \r\n")
                for i in range(len(raw)):
                    self._ser.write(f"{raw[i]}")
                    self._ser.write('   ')
                    for _ in range(int(calibrated[i]*10)):
                        self._ser.write('+')
                    self._ser.write("\r\n")

                self._ser.write(f"Measured value: {value:.2f}\r\n")
                """
                """
                self._ser.write("\r\nPlease Enter a Speed: \r\n->: ")
                value = yield from multichar_input(self._ser)
                """

                # BATTERY DROOP COMPENSATION
                """
                adcVal = self._battAdc.read()

                adcVoltage = (adcVal / 4096 * 3.3)

                # Scale for 4.7k and 10k voltage divider
                battVoltage = adcVoltage / 0.305

                # Calculate scaling factor
                effortScale = 6.5 / battVoltage

                self._ser.write(f'Voltage: {battVoltage}V, Scale: {effortScale}, output: {battVoltage * effortScale}\r\n')
                """

                # Return to main prompt
                self._ser.write(UI_prompt)
                self._state = S1_CMD

            # -----------------------
            # S8_CALIBRATE: calibration state
            # -----------------------
            elif self._state == S8_CALIBRATION:
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
                    self._ser.write(UI_prompt)
                    self._state = S1_CMD

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
                self._ser.write(UI_prompt)
                self._state = S1_CMD

            # -----------------------
            # S9_LINEFOLLOW: Line Following State
            # -----------------------
            elif self._state == S9_LINEFOLLOW:
                self._ser.write("Line Follow Mode\r\n")

                # Set sensor array into RUN mode
                self._reflectanceMode.put(3)

                # Configure motor control params
                self._leftMotorKi.put(0.6)
                self._rightMotorKi.put(0.6)

                self._leftMotorKp.put(0.02)
                self._rightMotorKp.put(0.02)

                self._leftMotorSetPoint.put(0)
                self._rightMotorSetPoint.put(0)

                self._leftMotorGo.put(1)
                self._rightMotorGo.put(1)

                # Enable line following controller
                self._lineFollowGo.put(1)

                self._ser.write("Line Following Started. Please Press Enter to Stop.\r\n")

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

                # Stop line-following mode and related tasks before returning.
                self._lineFollowGo.put(0)
                self._reflectanceMode.put(0)
                self._leftMotorGo.put(0)
                self._rightMotorGo.put(0)
                self._leftMotorSetPoint.put(0)
                self._rightMotorSetPoint.put(0)

                
                # Return to main prompt
                self._ser.write(UI_prompt)
                self._state = S1_CMD


            # Yield the current state per scheduler convention
            yield self._state
