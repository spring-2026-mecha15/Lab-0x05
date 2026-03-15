from constants import CSV_BEGIN, CSV_END


def run(ui):
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
