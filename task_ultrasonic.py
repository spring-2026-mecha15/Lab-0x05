"""
task_ultrasonic.py

Cooperative scheduler task that wraps the UltrasonicSensor driver.
On startup a 3-sample moving-average filter is enabled. Each scheduler
tick calls sensor.loop() to advance the non-blocking measurement state
machine, then publishes the latest distance (or 0 if no reading is
available) to a shared variable.
"""

from pyb import Pin
from drivers.ultrasonic import UltrasonicSensor
from task_share import Share


class task_ultrasonic:
    """
    Scheduler task that continuously updates the ultrasonic distance
    measurement and publishes the result to a share.
    """

    def __init__(self, sensor: UltrasonicSensor, distanceShare: Share):
        """
        Initialize the ultrasonic task and enable the sample filter.

        Parameters
        ----------
        sensor : UltrasonicSensor
            Instantiated ultrasonic sensor driver.
        distanceShare : Share
            Output share for the filtered distance reading (in the units
            returned by the driver). Written as 0 when no reading is
            available.
        """
        self._sensor = sensor
        self._distanceShare = distanceShare

        self._sensor.enable_filter(3)

    def run(self):
        """
        Cooperative function for scheduler
        """
        while True:
            self._sensor.loop()

            distance = self._sensor.get_distance()

            if distance is None:
                self._distanceShare.put(0)
            else:
                self._distanceShare.put(distance)

            yield
        

        