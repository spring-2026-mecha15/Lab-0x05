from constants import *
from pyb import Pin, ADC
import json
import utime

"!!!!!!Load calibration into memory, make non blocking if needed!!!!!!"


class Reflectance_Sensor:
    """
    Simple motor driver wrapper for a Romi-style motor driver.

    Responsibilities:
      - configure a PWM channel for the motor
      - control the direction pin
      - control the sleep (enable) pin
      - provide a simple set_effort(value) interface where value is in
        -100..100 (percent). Positive values select one direction,
        negative values select the other direction.

    Notes:
      - `pwm`/`slp`/`dir` arguments may be either Pin instances or arguments
        that the pyb.Pin constructor accepts
    """

    # def __init__(self, A1, A2, A3, A4, A5, A6, A7):

    #     # Configure pins as inputs
    #     self.sensor_1 = ADC(Pin(A1))
    #     self.sensor_2 = ADC(Pin(A2))
    #     self.sensor_3 = ADC(Pin(A3))
    #     self.sensor_4 = ADC(Pin(A4))
    #     self.sensor_5 = ADC(Pin(A5))
    #     self.sensor_6 = ADC(Pin(A6))
    #     self.sensor_7 = ADC(Pin(A7))

    def __init__(self, analogPins: list[Pin]):
        # Ordered tuple of ADC configured pins as inputs
        self._sensors = tuple([ADC(Pin(pin)) for pin in analogPins])

        # Number of sensors configured
        self._numSensors = len(self._sensors)

        # 2D-Array containing calibration data
        #  - self._calibration[0]: list of dark calibration values for each sensor
        #  - self._calibration[1]: list of light calibration values for each sensor
        self._calibration: list[list[float]]

    def get_values(self):

        #Probably need to load calibration values into memeory instead of reading from json each time (slows down alot)
        # Load calibration
        with open("ir_calibration.json", "r") as f:
            calibration = json.load(f)

        # Read raw sensors once
        # raw = [
        #     self.sensor_1.read(),
        #     self.sensor_2.read(),
        #     self.sensor_3.read(),
        #     self.sensor_4.read(),
        #     self.sensor_5.read(),
        #     self.sensor_6.read(),
        #     self.sensor_7.read()
        # ]
        raw = list(map(lambda x: x.read(), self._sensors))

        # List to store sensor readings with calibration applied
        calibrated = []

        for r, cal in zip(raw, calibration):
            dark = cal["dark"]
            light = cal["light"]

            if light != dark:
                # Linear interpolation from 2-pt calibration data
                value = (r - dark) / (light - dark)
                
                # Clip value between 0 and 1
                if value > 1:
                    value = 1
                elif value < 0:
                    value = 0
            else:
                value = 0

            calibrated.append(value)

        # Weighted line position
        # weights = [-3, -2, -1, 0, 1, 2, 3]
        # The line below should only be used for sensors arrays
        # with an odd number of sensors
        weights = list(range((-self._numSensors // 2) + 1, (self._numSensors // 2) + 1))
        total = sum(calibrated)

        # Compute weighted sensor values
        if total != 0:
            C = sum(w * v for w, v in zip(weights, calibrated)) / total
        else:
            C = 0

        return raw, calibrated, C

    def load_calibration_from_file(self, filename: str):
        with open(filename, "r") as fhand:
            calibration = json.load(fhand)

            print(calibration)

                # for i in range(self._numSensors):



    def calibrate(self, mode):  # "light" or "dark"

        #Might need to make this non blocking (wanted to test fucntionality first though)
        
        try:
            with open("ir_calibration.json", "r") as f:
                calibration = json.load(f)
        except:
            calibration = [{"light": 0, "dark": 0} for _ in range(self._numSensors)]

        sample_count = 200
        samples = [] 

        print()

        for i in range(sample_count):
        #   readings = [
        #       self.sensor_1.read(),
        #       self.sensor_2.read(),
        #       self.sensor_3.read(),
        #       self.sensor_4.read(),
        #       self.sensor_5.read(),
        #       self.sensor_6.read(),
        #       self.sensor_7.read()
        #   ]
            # readings = list(map(lambda x: x.read(), self._sensors))
            readings = list([x.read() for x in self._sensors])
            samples.append(readings)  # collect one sample set

            print(f'\r{i}', end='')
            
            # Calculate the ticks in ms before continuing to sample
            deadline_ms = utime.ticks_add(utime.ticks_ms(), 30)

            while True:
                if utime.ticks_diff(deadline_ms, utime.ticks_ms()) > 0:
                    yield
                else:
                    break

        for i in range(self._numSensors):
            avg = sum(sample[i] for sample in samples) / sample_count
            calibration[i][mode] = avg
            yield

        with open("ir_calibration.json", "w") as f:
            json.dump(calibration, f)
