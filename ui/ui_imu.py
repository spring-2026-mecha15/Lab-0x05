def run(ui):
    while True:
        # Redraw submenu each iteration
        ui._ser.write("\r\n")
        ui._ser.write("  +--------------------------------+\r\n")
        ui._ser.write("  | IMU Submenu                    |\r\n")
        ui._ser.write("  +--------------------------------+\r\n")
        ui._ser.write("  | 1: Load calibration            |\r\n")
        ui._ser.write("  | 2: Save calibration            |\r\n")
        ui._ser.write("  | 3: Tare accel and gyro         |\r\n")
        ui._ser.write("  | 4: Read and print values       |\r\n")
        ui._ser.write("  | 5: Get calibration states      |\r\n")
        ui._ser.write("  |                                |\r\n")
        ui._ser.write("  | 0: Exit                        |\r\n")
        ui._ser.write("  +--------------------------------+\r\n")
        ui._ser.write("  ->: ")

        while True:
            if ui._ser.any():
                inChar = ui._ser.read(1).decode('ascii')

                if inChar.isdigit():
                    user_sel = int(inChar)

                    # Load calibration from file
                    if user_sel == 0:
                        ui._ser.write(f"{inChar}\r\n")
                        return  # Exit to task_user, which sets S0_PROMPT

                    elif user_sel == 1:
                        ui._ser.write(f"{inChar}\r\n")
                        ui._imuMode.put(1)

                        while not ui._imuMode.get():
                            yield

                        ui._ser.write("Calibration loaded.\r\n")

                        break  # Redraw imu submenu

                    # Save calibration to file
                    elif user_sel == 2:
                        ui._ser.write(f"{inChar}\r\n")
                        ui._imuMode.put(2)

                        while not ui._imuMode.get():
                            yield
                        ui._ser.write("Calibration saved.\r\n")

                        break  # Redraw imu submenu

                    # Tare readings
                    elif user_sel == 3:
                        ui._ser.write(f"{inChar}\r\n")
                        ui._imuMode.put(3)

                        while not ui._imuMode.get():
                            yield

                        ui._ser.write("  Readings tared.\r\n")

                        break  # Redraw imu submenu

                    # Read and print
                    elif user_sel == 4:
                        ui._ser.write(f"{inChar}\r\n")
                        ui._imuMode.put(4)

                        while not ui._imuMode.get():
                            yield

                        heading = ui._imuHeading.get()
                        headingRate = ui._imuHeadingRate.get()

                        ui._ser.write("  IMU Data: \r\n")
                        ui._ser.write(f"   - heading: {heading}\r\n")
                        ui._ser.write(f"   - heading rate: {headingRate}\r\n")

                        break  # Redraw imu submenu

                    # Get calibration status
                    elif user_sel == 5:
                        ui._ser.write(f"{inChar}\r\n")
                        ui._imuMode.put(5)

                        while not ui._imuMode.get():
                            yield

                        raw_calib = ui._imuCalib.get()

                        s_sys = (raw_calib >> 6) & 0x03
                        s_gyro = (raw_calib >> 4) & 0x03
                        s_accel = (raw_calib >> 2) & 0x03
                        s_mag = raw_calib & 0x03

                        ui._ser.write("  Calibration status: \r\n")
                        ui._ser.write(f"   - System: {s_sys}\r\n")
                        ui._ser.write(f"   - Gyro:   {s_gyro}\r\n")
                        ui._ser.write(f"   - Accel:  {s_accel}\r\n")
                        ui._ser.write(f"   - Mag:    {s_mag}\r\n")

                        break  # Redraw imu submenu

            yield
