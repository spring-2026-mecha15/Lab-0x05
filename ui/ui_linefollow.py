"""
Line-following activation screen for the Romi robot UI.

Starts the autonomous line-following mode by enabling the reflectance sensor
array, both motors, and the line-following controller.  While running, sensor
telemetry is streamed between ``CSV_BEGIN`` / ``CSV_END`` markers so the host
can log data.  The user presses Enter to stop, at which point all actuators
and the controller are disabled and setpoints are zeroed.
"""
from constants import CSV_BEGIN, CSV_END


def run(ui):
    """
    Generator that manages the full lifecycle of one line-following session.

    Sequence:
        1. Enable reflectance sensor (mode 3) and both motors.
        2. Yield once to let the sensor task pick up the mode change.
        3. Enable the line-following controller.
        4. Write ``CSV_BEGIN`` marker and enter a non-blocking wait loop.
        5. On Enter / Return: break, write ``CSV_END``, disable everything.

    Args:
        ui: UI context object exposing ``_ser``, ``_reflectanceMode``,
            ``_leftMotorGo``, ``_rightMotorGo``, ``_lineFollowGo``,
            ``_leftMotorSetPoint``, and ``_rightMotorSetPoint`` shares/queues.

    Yields:
        int: ``0`` on every loop iteration to cooperate with the scheduler.
    """
    ui._ser.write("Line Follow Mode\r\n")

    # Set sensor array into RUN mode
    ui._reflectanceMode.put(3)
    ui._leftMotorGo.put(1)
    ui._rightMotorGo.put(1)
    yield  # Let sensor array pick up the change

    # Enable line following controller
    ui._lineFollowGo.put(1)

    ui._ser.write("Line Following Started. Please Press Enter to Stop.\r\n")

    # Wait for newline or carriage return, non-blocking (yielding)
    ui._ser.write(f"{CSV_BEGIN}\r\n")
    stream_decimation = 5
    sample_idx = 0
    while True:
        if ui._ser.any():
            inChar = ui._ser.read(1).decode()
            if inChar in {"\r", "\n"}:
                # Clear any remaining input then exit help
                while ui._ser.any():
                    ui._ser.read(1)
                break
        yield 0

    # Stop line-following mode and related tasks before returning.
    ui._ser.write(f"{CSV_END}\r\n")

    ui._lineFollowGo.put(0)
    ui._reflectanceMode.put(0)
    ui._leftMotorGo.put(0)
    ui._rightMotorGo.put(0)
    ui._leftMotorSetPoint.put(0)
    ui._rightMotorSetPoint.put(0)
