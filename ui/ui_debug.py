from pyb import ADC
from constants import BATT_ADC


def run(ui):
    ui._ser.write("DEBUG\r\n")

    ############################################
    # LINE SENSOR CENTROID VISUALIZATION
    ############################################
    raw, calibrated, value, found = ui._reflectanceSensor.get_values()
    ui._ser.write(f" RAW   CALIBRATED  \r\n")
    for i in range(len(raw)):
        ui._ser.write(f"{raw[i]:04d}")
        ui._ser.write('   ')
        for _ in range(int(calibrated[i]*10)):
            ui._ser.write('+')
        ui._ser.write("\r\n")

    ui._ser.write(f"Measured centroid: {value:.2f}\r\n")
    ui._ser.write(f"Line detected: {'TRUE' if found else 'FALSE'}\r\n\n")
    ############################################
    ############################################
    # BATTERY LEVEL
    ############################################
    # --- Scale value to account for battery droop
    adcVoltage = ADC(BATT_ADC).read() / 4096 * 3.3

    # Scale for 4.7k and 10k voltage divider.
    # (Slightly tweaked to account for actual VDD)
    battVoltage = adcVoltage / 0.305

    ui._ser.write(f"Battery voltage: {battVoltage:.2f}V\r\n\n")

    ############################################
    # BATTERY LEVEL
    ############################################
    distance = ui._ultrasonicDistance.get()
    ui._ser.write(f"Ultrasonic distance: {distance:.2f}cm\r\n")
    ############################################

    yield  # required to make this a generator
