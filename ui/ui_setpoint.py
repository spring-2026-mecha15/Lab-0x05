from multichar_input import multichar_input


def run(ui):
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
