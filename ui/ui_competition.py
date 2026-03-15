from utime import ticks_us, ticks_diff
from constants import CSV_END


def run(ui):
    ui._ser.write("State Estimation Test\r\n")

    ui._competitionGoFlag.put(1)

    ui._ser.write("Line Following Started. Please Press Enter to Stop.\r\n")

    stream_decimation = 5
    sample_idx = 0
    start_time = ticks_us()

    while True:
        # Check if observer has traveled X distance, then stop motors
        if ui._competitionGoFlag.get() == 0:
            break

        ui._ser.write(f"c: {ui._observerCenterDistance.get()}, s: {ui._lineFollowSetPoint.get():.2f}\r\n")

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

    ui._competitionGoFlag.put(0)
    ui._lineFollowGo.put(0)
    ui._reflectanceMode.put(0)
    ui._imuMode.put(0)
    ui._leftMotorGo.put(0)
    ui._rightMotorGo.put(0)
    ui._leftMotorSetPoint.put(0)
    ui._rightMotorSetPoint.put(0)
