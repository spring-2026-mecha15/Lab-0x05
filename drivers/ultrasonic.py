# DIYables_MicroPython_Ultrasonic_Sensor.py

from machine import Pin
import utime
import gc

class UltrasonicSensor:
    def __init__(self, trig_pin, echo_pin):
        self.trig = Pin(trig_pin, Pin.OUT)
        self.echo = Pin(echo_pin, Pin.IN)
        self.detection_threshold = float('inf')  # Initially set to infinity
        self.filter_enabled = False  # Filter is disabled by default
        self.num_samples = 1  # Default number of samples to 1 when filter is disabled
        self.distances = []  # Store measurements

    def loop(self):
        """Perform a measurement cycle and update the list of distances."""
        # Ensure the trigger pin is low for a clean pulse
        self.trig.value(0)
        utime.sleep_us(5)
        
        # Send a 10 microsecond pulse to start the measurement
        self.trig.value(1)
        utime.sleep_us(10)
        self.trig.value(0)
        
        start_time = utime.ticks_us()
        timeout = 10000  # Timeout in microseconds (e.g., 10 milliseconds ~ 170cm)

        # Wait for the echo to start
        while self.echo.value() == 0:
            if utime.ticks_diff(utime.ticks_us(), start_time) > timeout:
                return None  # Return None or some error indication on timeout

        # Record the start time of the echo
        signal_off = utime.ticks_us()

        # Wait for the echo to end
        while self.echo.value() == 1:
            if utime.ticks_diff(utime.ticks_us(), signal_off) > timeout:
                return None  # Return None or some error indication on timeout

        # Record the end time of the echo
        signal_on = utime.ticks_us()
            
        # Calculate the duration of the echo pulse
        time_passed = utime.ticks_diff(signal_on, signal_off)
        
        # Calculate the distance in centimeters
        distance = (time_passed * 0.017)
        self.distances.append(distance)
        if len(self.distances) > self.num_samples:
            self.distances.pop(0)  # Maintain a fixed length of distances

    def get_distance(self):
        """Return the calculated distance based on current measurements."""
        if not self.distances:
            return None  # Return None if no measurements have been made yet

        # Check if filtering is enabled and there are enough samples to filter
        if self.filter_enabled and len(self.distances) >= self.num_samples:
            sorted_distances = sorted(self.distances)
            # Consider only the middle samples if filtering is enabled
            mid_index_start = len(sorted_distances) // 4
            mid_index_end = len(sorted_distances) * 3 // 4
            mid_distances = sorted_distances[mid_index_start:mid_index_end]
            if not mid_distances:  # Check if the slice results in an empty list
                return None  # Return None or an appropriate default value to prevent division by zero
            calculated_distance = sum(mid_distances) / len(mid_distances)
        else:
            # Use the latest measurement if no filtering or not enough samples for filtering
            calculated_distance = self.distances[-1] if self.distances else None

        if calculated_distance is None:
            return None  # Safeguard against no available data to process

        # Compare against the detection threshold
        if calculated_distance > self.detection_threshold:
            return False  # Indicate no object detected within the threshold
        return calculated_distance

    def set_detection_threshold(self, distance):
        """Set the maximum distance beyond which no object is considered detected."""
        self.detection_threshold = distance

    def enable_filter(self, num_samples=20):
        """Enable filtering of measurements and set number of samples for filtering."""
        if num_samples > 0:
            self.num_samples = num_samples
            self.filter_enabled = True
        else:
            raise ValueError("Number of samples must be greater than 0")

    def disable_filter(self):
        """Disable filtering of measurements and reset number of samples to 1."""
        self.filter_enabled = False
        self.num_samples = 1  # Reset to default sample count


gc.collect()