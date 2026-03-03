from task_share import Share
from micropython import const
from drivers.imu import BNO055, NDOF_OP_MODE
from constants import IMU_FILE
from pyb import I2C
try:
    import ujson as json
except ImportError:
    import json


S0_BEGIN      = const(0)
S1_CALIBRATE  = const(1)
S2_IDLE       = const(2)
S3_RUN_NDOF   = const(3)
S4_SAVE_CALIB = const(4)
S5_LOAD_CALIB = const(5)
S6_TARE       = const(6)
S7_READ_VALS  = const(7)
S8_GET_CALIB_STATE = const(8)

class task_imu:
    def __init__(self, imuSensor: BNO055, mode: Share, calibration: Share,
                 accelX: Share, accelY: Share, accelZ: Share,
                 gyroX: Share, gyroY: Share, gyroZ: Share,
                 heading: Share, headingRate: Share
                 ):

        self._mode = mode
        self._calibration = calibration

        self._accelX = accelX 
        self._accelY = accelY
        self._accelZ = accelZ
        self._gyroX = gyroX
        self._gyroY = gyroY
        self._gyroZ = gyroZ
        self._heading = heading
        self._headingRate = headingRate

        self._imu = imuSensor

        self._calibration_saved = False

        print('IMU Task initialized')

        self._state = S0_BEGIN

    def _read_imu_file(self):
        try:
            with open(IMU_FILE, "r") as gains_file:
                data = json.load(gains_file)
        except (OSError, ValueError):
            data = {}

        if not isinstance(data, dict):
            data = {}

        return data

    def _write_imu_file(self, data):
        try:
            with open(IMU_FILE, "w") as gains_file:
                json.dump(data, gains_file)
        except OSError:
            return False
        return True

    def _save_calibration(self):
        offsets = yield from self._imu.get_calibration_offsets()
        accel_tare, gyro_tare = self._imu.get_tare()

        data = self._read_imu_file()

        data["imu"] = {
            "offsets": list(offsets),
            "tare": {
                "accel": list(accel_tare),
                "gyro": list(gyro_tare),
            },
        }

        return self._write_imu_file(data)

    def _save_tare_only(self):
        accel_tare, gyro_tare = self._imu.get_tare()

        data = self._read_imu_file()
        imu_data = data.get("imu", {}) if isinstance(data.get("imu", {}), dict) else {}
        imu_data["tare"] = {
            "accel": list(accel_tare),
            "gyro": list(gyro_tare),
        }
        data["imu"] = imu_data

        return self._write_imu_file(data)

    def _load_calibration(self):
        data = self._read_imu_file()

        imu_data = data.get("imu", {}) if isinstance(data, dict) else {}
        offsets = imu_data.get("offsets") if isinstance(imu_data, dict) else None

        tare_data = imu_data.get("tare") if isinstance(imu_data, dict) else None
        if isinstance(tare_data, dict):
            accel_tare = tare_data.get("accel")
            gyro_tare = tare_data.get("gyro")
            if accel_tare is not None and gyro_tare is not None:
                try:
                    self._imu.set_tare(accel_tare, gyro_tare)
                except (TypeError, ValueError, IndexError):
                    self._imu.clear_tare()

        if offsets is None:
            return False

        try:
            payload = bytes(offsets)
        except (TypeError, ValueError):
            return False

        if len(payload) != 22:
            return False

        yield from self._imu.set_calibration_offsets(payload)
        return True

    def tare_accel_gyro(self, sample_count=100, sample_delay_ms=5):
        yield from self._imu.tare_accel_gyro(sample_count=sample_count, sample_delay_ms=sample_delay_ms)
        return self._save_tare_only()

    def clear_tare(self):
        self._imu.clear_tare()
        return self._save_tare_only()

    def run(self):
        while True:
            if self._state == S0_BEGIN:
                # Begin sensor (contains cooperative delays)
                yield from self._imu.initialize(NDOF_OP_MODE)

                # Attempt to restore saved calibration; otherwise calibrate live.
                loaded = yield from self._load_calibration()
                if loaded:
                    # print('DEBUG: Calibration restored')
                    self._calibration_saved = True
                    self._state = S2_IDLE
                else:
                    # self._state = S1_CALIBRATE
                    self._state = S2_IDLE


            elif self._state == S1_CALIBRATE:
                # Get calibration states
                calib_status = self._imu.calibration_status()

                # Continue when all systems calibrated (status = 3)
                if all([x == 0x03 for x in calib_status]):
                    if not self._calibration_saved:
                        self._calibration_saved = bool((yield from self._save_calibration()))
                        # print('DEBUG: Calibration saved')
                    self._state = S2_IDLE

            elif self._state == S2_IDLE:
                mode = self._mode.get()

                if mode == 1:
                    self._state = S4_SAVE_CALIB
                elif mode == 2:
                    self._state = S5_LOAD_CALIB
                elif mode == 3:
                    self._state = S6_TARE
                elif mode == 4:
                    self._state = S7_READ_VALS
                elif mode == 5:
                    self._state = S8_GET_CALIB_STATE
                elif mode == 0xFF:
                    self._state = S3_RUN_NDOF


            elif self._state == S3_RUN_NDOF:
                gx, gy, gz = self._imu.gyro()
                ax, ay, az = self._imu.acceleration()
                mx, my, mz = self._imu.magnetic()

                self._gyroX.put(gx)
                self._gyroY.put(gy)
                self._gyroZ.put(gz)

                self._accelX.put(ax)
                self._accelY.put(ay)
                self._accelZ.put(az)

                # self._heading.put(mz)

                self._state = S2_IDLE
            
            elif self._state == S4_SAVE_CALIB:
                result = yield from self._save_calibration()

                self._mode.put(0) # Ack that save calibration is done

                self._state = S2_IDLE

            elif self._state == S5_LOAD_CALIB:
                result = yield from self._load_calibration()

                self._mode.put(0) # Ack that load calibration is done

                self._state = S2_IDLE

            elif self._state == S6_TARE:
                result = yield from self.tare_accel_gyro()
                self._mode.put(0)

                self._state = S2_IDLE

            elif self._state == S7_READ_VALS:
                gx, gy, gz = self._imu.gyro()
                ax, ay, az = self._imu.acceleration()
                mx, my, mz = self._imu.magnetic()

                self._gyroX.put(gx)
                self._gyroY.put(gy)
                self._gyroZ.put(gz)

                self._accelX.put(ax)
                self._accelY.put(ay)
                self._accelZ.put(az)

                self._mode.put(0)

                self._state = S2_IDLE

            elif self._state == S8_GET_CALIB_STATE:
                s_sys, s_gyro, s_accel, s_mag = self._imu.calibration_status()

                result = 0
                result |= (s_sys << 6)
                result |= (s_gyro << 4)
                result |= (s_accel << 2)
                result |= s_mag

                self._calibration.put(result)

                self._mode.put(0)

                self._state = S2_IDLE


            yield self._state