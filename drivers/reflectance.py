from constants import *
from pyb import Pin, ADC
import json
import utime


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

    def __init__(self, A1, A2, A3, A4, A5, A6, A7):

        # Configure pins as inputs
        self.sensor_1 = ADC(Pin(A1))
        self.sensor_2 = ADC(Pin(A2))
        self.sensor_3 = ADC(Pin(A3))
        self.sensor_4 = ADC(Pin(A4))
        self.sensor_5 = ADC(Pin(A5))
        self.sensor_6 = ADC(Pin(A6))
        self.sensor_7 = ADC(Pin(A7))

    def get_values(self):

        # Load calibration
        with open("ir_calibration.json", "r") as f:
            calibration = json.load(f)

        calibrated = []

        # Read raw sensors once
        raw = [
            self.sensor_1.read(),
            self.sensor_2.read(),
            self.sensor_3.read(),
            self.sensor_4.read(),
            self.sensor_5.read(),
            self.sensor_6.read(),
            self.sensor_7.read()
        ]

        for r, cal in zip(raw, calibration):
            dark = cal["dark"]
            light = cal["light"]

            if light != dark:
                value = (r - dark) / (light - dark)
                value = max(0, min(1, value))
            else:
                value = 0

            calibrated.append(value)

        # Weighted line position
        weights = [-3, -2, -1, 0, 1, 2, 3]
        total = sum(calibrated)

        if total != 0:
            C = sum(w * v for w, v in zip(weights, calibrated)) / total
        else:
            C = 0

        return raw, calibrated, C


    def calibrate(self, mode):  # "light" or "dark"
        try:
            with open("ir_calibration.json", "r") as f:
                calibration = json.load(f)
        except:
            calibration = [{"light": 0, "dark": 0} for _ in range(7)]

        sample_count = 200
        samples = [] 

        while len(samples) < sample_count:
          readings = [
              self.sensor_1.read(),
              self.sensor_2.read(),
              self.sensor_3.read(),
              self.sensor_4.read(),
              self.sensor_5.read(),
              self.sensor_6.read(),
              self.sensor_7.read()
          ]
          samples.append(readings)  # collect one sample set
          utime.sleep_ms(30)   # wait 30 ms before next sample

        for i in range(7):
            avg = sum(sample[i] for sample in samples) / len(samples)
            calibration[i][mode] = avg

        with open("ir_calibration.json", "w") as f:
            json.dump(calibration, f)
