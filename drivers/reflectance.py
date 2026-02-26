"""Reflectance sensor array driver with file-backed calibration support."""

from pyb import Pin, ADC
import json
import utime


class Reflectance_Sensor:
    # Driver for a reflectance sensor array read through ADC channels.

    # The class provides:
    #     - one-shot raw + normalized sensor reads (`get_values`)
    #     - calibration file loading (`load_calibration_from_file`)
    #     - cooperative calibration sampling (`calibrate`) via `yield`
    

    _CALIBRATION_FILE = "ir_calibration.json"
    _SAMPLE_COUNT = 100
    _SAMPLE_PERIOD_MS = 10
    _READ_SAMPLES = 5  # Number of raw reads to average in get_values()
    
    # Threshold-based line loss detection (sum of calibrated values)
    _LINE_DETECT_THRESHOLD_LOW = 0.5   # Below this = all black (line lost)
    _LINE_DETECT_THRESHOLD_HIGH = 6.85  # Above this = all white (line lost)
    # For 7 sensors: valid range roughly 0.15 to 6.85 (ensures some contrast)

    def __init__(self, analogPins: list[Pin]):
        """Create ADC objects for each analog reflectance sensor pin."""
        self._sensors = tuple(ADC(Pin(pin)) for pin in analogPins)

        # Number of sensors configured
        self._numSensors = len(self._sensors)
        
        # Track last valid centroid for line loss recovery
        self._last_valid_centroid = 0.0

        # Load calibration from file into memory (cached for fast access)
        self._calibration = self._load_calibration_dicts(self._CALIBRATION_FILE)

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
        # Return `(raw, calibrated, C, line_detected)` where:
        # - `raw`: list of averaged raw ADC values
        # - `calibrated`: list of normalized sensor values (0=white, 1=black)
        # - `C`: weighted line position (centroid)
        # - `line_detected`: True if line is detected, False if lost (noise/uniform)
        
        # Raw values are averaged over `_READ_SAMPLES` reads to reduce noise.
        # When line is lost, `C` returns the last valid centroid.
        

        # Use cached calibration from memory (loaded during __init__)
        calibration = self._calibration

        # Average multiple raw reads to reduce noise
        raw_samples = [self._read_raw() for _ in range(self._READ_SAMPLES)]
        raw = [
            sum(sample[i] for sample in raw_samples) // self._READ_SAMPLES
            for i in range(self._numSensors)
        ]

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

        # Threshold-based line loss detection
        line_detected = (self._LINE_DETECT_THRESHOLD_LOW <= total <= self._LINE_DETECT_THRESHOLD_HIGH)

        # print('Line found' if line_detected else 'Line not found', f'({total})')

        # Compute weighted sensor values
        if total != 0 and line_detected:
            centroid = sum(w * v for w, v in zip(weights, calibrated)) / total
            self._last_valid_centroid = centroid  # Store for line loss recovery
        else:
            # Line lost: return last valid centroid
            centroid = self._last_valid_centroid

        # return raw, calibrated, C, line_detected
        return raw, calibrated, centroid

    def get_centroid(self):
        return self.get_values()[2]

    def load_calibration_from_file(self, filename: str):
        # Load calibration file into `self._calibration`.

        # The on-disk format must be a list of dicts with 'dark' and
        # 'light' keys (the format produced by `calibrate`).
        
        with open(filename, "r") as fhand:
            calibration = json.load(fhand)

        self._calibration = calibration

        print(self._calibration)

    def calibrate(self, mode):  # "light" or "dark"
        # Cooperative calibration generator.

        # `mode` should be `"light"` or `"dark"`. The routine collects
        # `_SAMPLE_COUNT` raw samples and stores the per-sensor average to file.
        # It yields frequently so other cotasks can continue running.
        
        # After calibration completes, the in-memory cache is updated.
        
        calibration = self._load_calibration_dicts(self._CALIBRATION_FILE)

        sample_count = self._SAMPLE_COUNT
        samples = [] 

        for i in range(sample_count):
            readings = self._read_raw()
            samples.append(readings)  # collect one sample set

            # Calculate the ticks in ms before continuing to sample
            deadline_ms = utime.ticks_add(utime.ticks_ms(), self._SAMPLE_PERIOD_MS)
            
            while utime.ticks_diff(deadline_ms, utime.ticks_ms()) > 0:
                yield i

        for i in range(self._numSensors):
            avg = sum(sample[i] for sample in samples) / sample_count
            calibration[i][mode] = avg

        with open(self._CALIBRATION_FILE, "w") as f:
            json.dump(calibration, f)
        
        # Update the in-memory cache so get_values() uses the new calibration
        self._calibration = calibration
