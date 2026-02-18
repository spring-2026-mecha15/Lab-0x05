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
from task_share import Share, Queue
import micropython
from multichar_input import multichar_input
from constants import CSV_BEGIN, CSV_END
from drivers.reflectance import Reflectance_Sensor

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

# Reusable UI text
UI_prompt = """\r\n\n
+-----------------------------------------------------------------------+\r
| ME 405 Romi Tuning Interface Help Menu                                |\r
+---+-------------------------------------------------------------------+\r
| h | Print help menu                                                   |\r
| k | Enter new gain values                                             |\r
| s | Choose a new setpoint                                             |\r
| g | Trigger step response and print results                           |\r
| c | Line Sensor Calibration.                                          |\r
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
        reflectanceSensor,     # type: Reflectance_Sensor
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

        # Queues used for data collection / logging
        self._dataValues = dataValues               # type: Queue
        self._timeValues = timeValues               # type: Queue
        self._reflectanceSensor = reflectanceSensor # type: Reflectance_Sensor

        # Notify user task instantiation (kept from original behavior)
        self._ser.write("User Task object instantiated")

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
            # S3_GAINS: prompt for Kp then Ki (uses multichar_input)
            # -----------------------
            elif self._state == S3_GAINS:
                # Prompt and read new Kp
                self._ser.write(
                    f"Current Kp gains: L:{self._leftMotorKp.get():.2f}, R:{self._rightMotorKp.get():.2f}\r\n"
                )
                self._ser.write("Enter a new Kp gain value: \r\n->: ")
                value = yield from multichar_input(self._ser)

                if value is not None:
                    if value <= 0:
                        self._ser.write("Invalid Kp gain (<= 0). Kp gain unchanged")
                    else:
                        # Apply same Kp to both motors (preserved behavior)
                        self._leftMotorKp.put(value)
                        self._rightMotorKp.put(value)
                        self._ser.write(f"Kp Gain Set To: {value:.2f}\r\n")
                else:
                    self._ser.write("No value entered. Kp gains unchanged.\r\n")

                # Prompt and read new Ki
                self._ser.write(
                    f"\r\nCurrent Ki gains: L:{self._leftMotorKi.get():.2f}, R:{self._rightMotorKi.get():.2f}\r\n"
                )
                self._ser.write("Enter a new Ki gain value:\r\n->: ")
                value = yield from multichar_input(self._ser)

                if value is not None:
                    if value < 0:
                        self._ser.write("Invalid Ki gain (< 0). Ki gains unchanged")
                    else:
                        self._leftMotorKi.put(value)
                        self._rightMotorKi.put(value)
                        self._ser.write(f"Ki Gain Set To: {value} \r\n")
                else:
                    self._ser.write("No value entered. Ki gains unchanged.\r\n")

                # Show final values and return to prompt
                self._ser.write("\r\nController gains:\r\n")
                self._ser.write(f" - Kp: {self._leftMotorKp.get():.2f}\r\n")
                self._ser.write(f" - Ki: {self._leftMotorKi.get():.2f}")
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

                # When both go flags clear, collection is finished
                if not self._leftMotorGo.get() and not self._rightMotorGo.get():
                    self._ser.write("Data collection complete...\r\n")
                    self._ser.write("Printing data...\r\n")
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

                
                values = self._reflectanceSensor.get_values()
                self._ser.write("{}\r\n".format(values))

                # Return to main prompt
                self._ser.write(UI_prompt)
                self._state = S1_CMD

            # -----------------------
            # S8_CALIBRATE: calibraiton state
            # -----------------------
            elif self._state == S8_CALIBRATION:
                self._ser.write("CALIBRATION\r\n")
                self._ser.write("Would you like to begin calibration (y/n?\r\n")


                self._ser.write("Please Place Roami On a Light Surface Press Enter When Complete\r\n")

                while True:
                    if self._ser.any():
                        inChar = self._ser.read(1).decode()
                        if inChar in {"\r", "\n"}:
                            # Clear any remaining input then exit help
                            while self._ser.any():
                                self._ser.read(None)
                            break
                    yield

                self._reflectanceSensor.calibrate("light")

                self._ser.write("Please Place Roami On a Dark Surface Press Enter When Complete\r\n")

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

                self._reflectanceSensor.calibrate("dark")



                # Return to main prompt
                self._ser.write(UI_prompt)
                self._state = S1_CMD

            # Yield the current state per scheduler convention
            yield self._state
