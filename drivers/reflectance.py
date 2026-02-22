"""Reflectance sensor array driver with file-backed calibration support."""

from pyb import Pin, ADC
import json
import utime


class Reflectance_Sensor:
    """
        Driver for a reflectance sensor array read through ADC channels.

        The class provides:
            - one-shot raw + normalized sensor reads (`get_values`)
            - calibration file loading (`load_calibration_from_file`)
            - cooperative calibration sampling (`calibrate`) via `yield`
    """

    _CALIBRATION_FILE = "ir_calibration.json"
    _SAMPLE_COUNT = 200
    _SAMPLE_PERIOD_MS = 30

    def __init__(self, analogPins: list[Pin]):
        """Create ADC objects for each analog reflectance sensor pin."""
        self._sensors = tuple(ADC(Pin(pin)) for pin in analogPins)

        # Number of sensors configured
        self._numSensors = len(self._sensors)

        # In-memory [dark, light] pairs loaded from file when requested.
        self._calibration: list[list[float]] = []

    def _read_raw(self) -> list[int]:
        """Read all sensors once and return raw ADC values."""
        return [sensor.read() for sensor in self._sensors]

    def _load_calibration_dicts(self, filename: str) -> list[dict]:
        """Load calibration dictionaries from JSON, or create defaults if missing."""
        try:
            with open(filename, "r") as fhand:
                calibration = json.load(fhand)
        except:
            calibration = [{"light": 0, "dark": 0} for _ in range(self._numSensors)]

        return calibration

    @staticmethod
    def _clamp_01(value: float) -> float:
        """Clamp a value to the inclusive range [0, 1]."""
        if value > 1:
            return 1
        if value < 0:
            return 0
        return value

    def get_values(self):
        """Return `(raw, calibrated, C)` where `C` is weighted line position."""

        # Read calibration from JSON file.
        calibration = self._load_calibration_dicts(self._CALIBRATION_FILE)

        # Read raw sensors once
        raw = self._read_raw()

        # List to store sensor readings with calibration applied
        calibrated = []

        for r, cal in zip(raw, calibration):
            dark = cal["dark"]
            light = cal["light"]

            if light != dark:
                # Linear interpolation from 2-pt calibration data
                value = (r - dark) / (light - dark)
                value = self._clamp_01(value)
            else:
                value = 0

            calibrated.append(value)

        # Weighted line position
        # This weighting assumes a symmetric, odd-count sensor array.
        weights = list(range((-self._numSensors // 2) + 1, (self._numSensors // 2) + 1))
        total = sum(calibrated)

        # Compute weighted sensor values
        if total != 0:
            C = sum(w * v for w, v in zip(weights, calibrated)) / total
        else:
            C = 0

        return raw, calibrated, C

    def load_calibration_from_file(self, filename: str):
        """Load calibration file into `self._calibration` as `[dark, light]` pairs."""
        with open(filename, "r") as fhand:
            calibration = json.load(fhand)

            self._calibration = []
            for cal_pair in calibration:
                self._calibration.append([
                    cal_pair['dark'],
                    cal_pair['light']
                ])

    def calibrate(self, mode):  # "light" or "dark"
        """
        Cooperative calibration generator.

        `mode` should be `"light"` or `"dark"`. The routine collects
        `_SAMPLE_COUNT` raw samples and stores the per-sensor average to file.
        It yields frequently so other cotasks can continue running.
        """
        calibration = self._load_calibration_dicts(self._CALIBRATION_FILE)

        sample_count = self._SAMPLE_COUNT
        samples = [] 

        print()

        for i in range(sample_count):
            readings = self._read_raw()
            samples.append(readings)  # collect one sample set

            # Carriage return updates a single on-screen progress line.
            print(f'\r{i}', end='')
            
            # Calculate the ticks in ms before continuing to sample
            deadline_ms = utime.ticks_add(utime.ticks_ms(), self._SAMPLE_PERIOD_MS)

            while utime.ticks_diff(deadline_ms, utime.ticks_ms()) > 0:
                yield

        for i in range(self._numSensors):
            avg = sum(sample[i] for sample in samples) / sample_count
            calibration[i][mode] = avg
            yield

        with open(self._CALIBRATION_FILE, "w") as f:
            json.dump(calibration, f)
