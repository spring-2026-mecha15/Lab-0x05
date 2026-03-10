from pyb import Pin
from drivers.ultrasonic import UltrasonicSensor
from task_share import Share


class task_ultrasonic:
    def __init__(self, sensor: UltrasonicSensor, distanceShare: Share):
        self._sensor = sensor
        self._distanceShare = distanceShare

        self._sensor.enable_filter(3)

    def run(self):
        while True:
            self._sensor.loop()

            distance = self._sensor.get_distance()

            if distance is None:
                self._distanceShare.put(0)
            else:
                self._distanceShare.put(distance)

            yield
        

        