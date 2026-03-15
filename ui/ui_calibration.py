def run(ui):
    ui._ser.write("CALIBRATION\r\n")

    # Double check this is the action the user wants to make
    ui._ser.write("Would you like to begin calibration (y/n)?\r\n")

    # Check for a valid keystroke
    while True:
        if ui._ser.any():
            inChar = ui._ser.read(1).decode()
            if inChar in {"y", "Y", "n", "N"}:
                break

    # If NO selected, return to main menu from this point
    if inChar in {"n", "N"}:
        yield 0
        return

    # Begin with light side calibration
    ui._ser.write("Place Romi on a light surface. Press Enter when ready\r\n")
    # Wait for ENT (cooperative)
    while True:
        if ui._ser.any():
            inChar = ui._ser.read(1).decode()
            if inChar in {"\r", "\n"}:
                # Clear any remaining input then exit help
                while ui._ser.any():
                    ui._ser.read(1)
                break
        yield 0

    # Request that reflectance sensor goes into light calib. mode
    ui._reflectanceMode.put(2)

    # Wait for 'ack' that calibration is done (mode = 0)
    while ui._reflectanceMode.get() != 0:
        yield 0

    # Then perform dark side calibration
    ui._ser.write("Please Place Romi On a Dark Surface. Press Enter When Complete\r\n")

    # Wait for ENT (cooperative)
    while True:
        if ui._ser.any():
            inChar = ui._ser.read(1).decode()
            if inChar in {"\r", "\n"}:
                # Clear any remaining input then exit help
                while ui._ser.any():
                    ui._ser.read(1)
                break
        yield 0

    # Request that reflectance sensor goes into light calib. mode
    ui._reflectanceMode.put(1)

    # Wait for 'ack' that calibration is done (mode = 0)
    while ui._reflectanceMode.get() != 0:
        yield 0

    ui._ser.write('Sensor calibration complete.\r\n')
