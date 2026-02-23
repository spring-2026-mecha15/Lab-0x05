from task_share import Share
from drivers.reflectance import Reflectance_Sensor
import micropython

S0_IDLE        = micropython.const(0)
S1_CALIB_DARK  = micropython.const(1)
S2_CALIB_LIGHT = micropython.const(2)
S3_RUN         = micropython.const(3)

class task_reflectance:
    def __init__(
            self,
            reflectanceSensor: Reflectance_Sensor,
            reflectanceMode: Share,
            lineCentroid:  Share,
            lineFound:     Share
        ):
        
        self._state = 0

        self._sensor = reflectanceSensor

        self._mode = reflectanceMode

        self._lineCentroid = lineCentroid

        self._lineFound = lineFound

        print("Reflectance sensor instantiated")

    def run(self):
        while True:
            if self._state == S0_IDLE:
                mode = self._mode.get()

                if mode == 0:
                    pass
                elif mode == 1:
                    self._state = S1_CALIB_DARK
                elif mode == 2:
                    self._state = S2_CALIB_LIGHT
                elif mode == 3:
                    self._state = S3_RUN
                
                else:
                    raise ValueError(f"Invalid mode: {mode}")


            elif self._state == S1_CALIB_DARK:
                # Run calibration sequence (cooperatively) for dark readings
                cal_gen = self._sensor.calibrate("dark")
                while True:
                    try:
                        # print(next(cal_gen))
                        next(cal_gen)
                        yield 0

                    # When calibration is done, reset states
                    except StopIteration:
                        break
                    
                self._mode.put(0)
                self._state = S0_IDLE


            elif self._state == S2_CALIB_LIGHT:
                # Run calibration sequence (cooperatively) for light readings
                cal_gen = self._sensor.calibrate("light")
                while True:
                    try:
                        # print(next(cal_gen))
                        next(cal_gen)
                        yield 0

                    # When calibration is done, reset states
                    except StopIteration:
                        break

                self._mode.put(0)
                self._state = S0_IDLE


            elif self._state == S3_RUN:
                # First, test flag for exit
                if self._mode.get() == 0:
                    self._state = S0_IDLE

                # If running, get latest centroid and put into share
                centroid = self._sensor.get_centroid()
                self._lineCentroid.put(centroid)


            yield self._state