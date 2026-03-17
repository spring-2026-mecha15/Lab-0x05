"""
Controller gain tuning wizard for the Romi robot UI.

Steps the user through adjusting five control parameters via the serial
terminal, then persists all gains to ``GAINS_FILE`` so they survive a
power cycle.  Uses ``multichar_input`` for non-blocking numeric entry.

Parameters tuned (in order):
    1. Motor Kp  – proportional gain (must be > 0; same value applied to both motors)
    2. Motor Ki  – integral gain (must be ≥ 0)
    3. LF Kp     – line-follower proportional gain (must be ≥ 0)
    4. LF Ki     – line-follower integral gain (must be ≥ 0)
    5. LF Kff    – line-follower feed-forward gain (must be ≥ 0)
"""
from multichar_input import multichar_input
from constants import GAINS_FILE


def run(ui):
    """
    Generator that runs the interactive gain-tuning wizard.

    For each gain the function displays the current value, prompts for a new
    one, validates the range, and either updates the shared variable or
    reports why the entry was rejected.  A summary table is printed at the
    end and all gains are saved to disk.

    Uses ``yield from multichar_input(ui._ser)`` so the scheduler remains
    responsive while waiting for the user to type a value.

    Args:
        ui: UI context object exposing ``_ser``, ``_leftMotorKp``,
            ``_rightMotorKp``, ``_leftMotorKi``, ``_rightMotorKi``,
            ``_lineFollowKp``, ``_lineFollowKi``, ``_lineFollowKff``
            shares/queues, and a ``_save_gains()`` method.

    Yields:
        Delegates to ``multichar_input`` which yields on every serial poll.
    """
    # Prompt and read new motor Kp (assumes same gain for L/R)
    ui._ser.write(
        f"Current motor Kp gain: {ui._leftMotorKp.get():.2f}\r\n"
    )
    ui._ser.write("Enter a new motor Kp gain value: \r\n->: ")
    value = yield from multichar_input(ui._ser)

    if value is not None:
        if value <= 0:
            ui._ser.write("Invalid motor Kp gain (<= 0). Motor Kp gain unchanged")
        else:
            # Apply same Kp to both motors (preserved behavior)
            ui._leftMotorKp.put(value)
            ui._rightMotorKp.put(value)
            ui._ser.write(f"Motor Kp Gain Set To: {value:.2f}\r\n")
    else:
        ui._ser.write("No value entered. Motor Kp gains unchanged.\r\n")

    # Prompt and read new motor Ki (assumes same gain for L/R)
    ui._ser.write(
        f"\r\nCurrent motor Ki gains: {ui._leftMotorKi.get():.2f}\r\n"
    )
    ui._ser.write("Enter a new motor Ki gain value:\r\n->: ")
    value = yield from multichar_input(ui._ser)

    if value is not None:
        if value < 0:
            ui._ser.write("Invalid motor Ki gain (< 0). Motor Ki gains unchanged")
        else:
            ui._leftMotorKi.put(value)
            ui._rightMotorKi.put(value)
            ui._ser.write(f"Motor Ki Gain Set To: {value} \r\n")
    else:
        ui._ser.write("No value entered. Motor Ki gains unchanged.\r\n")

    # Prompt and read new line follower Kp
    ui._ser.write(
        f"\r\nCurrent line follower (LF) Kp gain: {ui._lineFollowKp.get():.2f}\r\n"
    )
    ui._ser.write("Enter a new LF Kp gain value:\r\n->: ")
    value = yield from multichar_input(ui._ser)

    if value is not None:
        if value < 0:
            ui._ser.write("Invalid LF Kp gain (< 0). LF Kp gains unchanged")
        else:
            ui._lineFollowKp.put(value)
            ui._ser.write(f"LF Kp Gain Set To: {value}\r\n")
    else:
        ui._ser.write("No value entered. LF Kp gains unchanged.\r\n")

    # Prompt and read new line follower Ki
    ui._ser.write(
        f"\r\nCurrent line follower (LF) Ki gain: {ui._lineFollowKi.get():.2f}\r\n"
    )
    ui._ser.write("Enter a new LF Ki gain value:\r\n->: ")
    value = yield from multichar_input(ui._ser)

    if value is not None:
        if value < 0:
            ui._ser.write("Invalid LF Ki gain (< 0). LF Ki gains unchanged")
        else:
            ui._lineFollowKi.put(value)
            ui._ser.write(f"LF Ki Gain Set To: {value}\r\n")
    else:
        ui._ser.write("No value entered. LF Ki gains unchanged.\r\n")

    # Prompt and read new LF feed forward gain
    ui._ser.write(
        f"\r\nCurrent LF Kff gain: {ui._lineFollowKff.get():.2f}\r\n"
    )
    ui._ser.write("Enter a new LF Kff gain value:\r\n->: ")
    value = yield from multichar_input(ui._ser)

    if value is not None:
        if value < 0:
            ui._ser.write("Invalid LF Kff gain (< 0). LF Kff gain unchanged")
        else:
            ui._lineFollowKff.put(value)
            ui._ser.write(f"LF Kff Gain Set To: {value}\r\n")
    else:
        ui._ser.write("No value entered. LF Kff gain unchanged.\r\n")

    # Show final values and return to prompt
    ui._ser.write("\r\nController gains:\r\n")
    ui._ser.write(f" - Motor Kp: {ui._leftMotorKp.get():.2f}\r\n")
    ui._ser.write(f" - Motor Ki: {ui._leftMotorKi.get():.2f}\r\n")
    ui._ser.write(f" - Line follower Kp: {ui._lineFollowKp.get():.2f}\r\n")
    ui._ser.write(f" - Line follower Ki: {ui._lineFollowKi.get():.2f}\r\n")
    ui._ser.write(f" - Line follower Kff: {ui._lineFollowKff.get():.2f}\r\n")
    if not ui._save_gains():
        ui._ser.write(f"Warning: failed to save {GAINS_FILE}.\r\n")
