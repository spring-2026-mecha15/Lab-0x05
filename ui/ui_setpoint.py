"""
Motor and line-follower setpoint configuration screen for the Romi robot UI.

Prompts the user to enter two setpoint values via the serial terminal:

    1. **Motor setpoint** – target wheel speed applied equally to both motors.
    2. **Line-follower (LF) setpoint** – the reference value used by the
       line-following proportional controller.

Changes are persisted to non-volatile storage via ``ui._save_gains()`` so
the setpoints are restored on the next power-up.
"""
from multichar_input import multichar_input


def run(ui):
    """
    Generator that prompts for and applies motor and LF setpoints.

    Displays the current value of each setpoint, accepts a new float via
    ``multichar_input``, and writes the result to the appropriate shared
    variable.  If the user presses Enter without typing a value the existing
    setpoint is left unchanged.

    Args:
        ui: UI context object exposing ``_ser``, ``_leftMotorSetPoint``,
            ``_rightMotorSetPoint``, ``_lineFollowSetPoint`` shares/queues,
            and a ``_save_gains()`` method.

    Yields:
        Delegates to ``multichar_input`` which yields on every serial poll.
    """
    ui._ser.write(f"\r\nCurrent motor setpoint: {ui._leftMotorSetPoint.get():.2f}\r\n")
    ui._ser.write("\r\nEnter new motor setpoint: \r\n->: ")
    value = yield from multichar_input(ui._ser)

    if value is not None:
        ui._leftMotorSetPoint.put(value)
        ui._rightMotorSetPoint.put(value)
        ui._ser.write(f"\r\nSetpoint Set To: {value:.2f}")
    else:
        ui._ser.write("\r\nNo setpoint specified. Value unchanged.")

    ui._ser.write(f"\r\nCurrent line follow setpoint: {ui._lineFollowSetPoint.get():.2f}\r\n")
    ui._ser.write("\r\nEnter new LF setpoint: \r\n->: ")
    value = yield from multichar_input(ui._ser)

    if value is not None:
        ui._lineFollowSetPoint.put(value)
        ui._ser.write(f"\r\nLF Setpoint Set To: {value:.2f}")
    else:
        ui._ser.write("\r\nNo LF setpoint specified. Value unchanged.")

    ui._save_gains()
