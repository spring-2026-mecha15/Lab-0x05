"""
task_imu.py

Cooperative scheduler task that manages the BNO055 IMU sensor.
The task moves through the following states:

  - S0_BEGIN:          Initialize the IMU and attempt to restore a saved
                       calibration from flash.
  - S1_CALIBRATE:      Wait until all subsystems reach full calibration,
                       then save offsets to flash.
  - S2_IDLE:           Dispatch mode commands (load/save calibration, tare,
                       read values, get calibration status, or run NDOF).
  - S3_RUN_NDOF:       Read heading and heading-rate from the IMU and publish
                       them to shares, then return to idle.
  - S4_SAVE_CALIB:     Persist current calibration offsets and tare values.
  - S5_LOAD_CALIB:     Restore calibration offsets and tare values from flash.
  - S6_TARE:           Collect accelerometer/gyro tare samples and save them.
  - S7_READ_VALS:      Perform a one-shot read of heading and heading-rate.
  - S8_GET_CALIB_STATE: Pack calibration sub-system statuses into a single
                        byte and publish it to a share.
"""

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
    """
    Scheduler task that initializes, calibrates, and reads the BNO055 IMU.
    Heading and heading-rate are published to shared variables for use by
    other tasks (e.g., closed-loop motor control).
    """

    def __init__(self, imuSensor: BNO055, mode: Share, calibration: Share,
                 heading: Share, headingRate: Share
                 ):
        """
        Initialize the IMU task.

        Parameters
        ----------
        imuSensor : BNO055
            Instantiated BNO055 driver.
        mode : Share
            Command share that selects the operating mode:
            0 = idle, 1 = load calibration, 2 = save calibration,
            3 = tare, 4 = read values, 5 = get calibration status,
            0xFF = run NDOF (continuous heading updates).
        calibration : Share
            Output share for the packed calibration status byte
            (sys[7:6] | gyro[5:4] | accel[3:2] | mag[1:0]).
        heading : Share
            Output share for the current Euler heading angle (degrees).
        headingRate : Share
            Output share for the current yaw rate from the gyroscope
            (degrees per second).
        """

        self._mode = mode
        self._calibration = calibration

        self._heading = heading
        self._headingRate = headingRate

        self._imu = imuSensor

        self._calibration_saved = False
        self._run_mode_active = False

        print('IMU Task initialized')

        self._state = S0_BEGIN

    def _read_imu_file(self):
        """
        Read and parse the IMU JSON file from flash.

        Returns
        -------
        dict
            Parsed file contents, or an empty dict if the file is missing
            or contains invalid JSON.
        """
        try:
            with open(IMU_FILE, "r") as gains_file:
                data = json.load(gains_file)
        except (OSError, ValueError):
            data = {}

        if not isinstance(data, dict):
            data = {}

        return data

    def _write_imu_file(self, data):
        """
        Serialize and write data to the IMU JSON file on flash.

        Parameters
        ----------
        data : dict
            Data to serialize and persist.

        Returns
        -------
        bool
            True on success, False if an OSError occurs during the write.
        """
        try:
            with open(IMU_FILE, "w") as gains_file:
                json.dump(data, gains_file)
        except OSError:
            return False
        return True

    def _save_calibration(self):
        """
        Retrieve current calibration offsets and tare values from the IMU,
        then persist them to the IMU JSON file.

        This is a generator method; use ``yield from _save_calibration()``.

        Returns
        -------
        bool
            True if the file was written successfully, False otherwise.
        """
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
        """
        Persist only the current tare values (accelerometer and gyroscope)
        to the IMU JSON file, leaving any stored calibration offsets intact.

        Returns
        -------
        bool
            True if the file was written successfully, False otherwise.
        """
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
        """
        Load calibration offsets and tare values from the IMU JSON file and
        apply them to the sensor.

        This is a generator method; use ``yield from _load_calibration()``.

        Returns
        -------
        bool
            True if valid 22-byte calibration offsets were found and applied,
            False if the file is missing, malformed, or the offset payload is
            the wrong length.
        """
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
        """
        Collect tare samples from the accelerometer and gyroscope, then
        save the resulting tare offsets to flash.

        This is a generator method; use ``yield from tare_accel_gyro()``.

        Parameters
        ----------
        sample_count : int, optional
            Number of samples to average for the tare (default 100).
        sample_delay_ms : int, optional
            Delay in milliseconds between samples (default 5).

        Returns
        -------
        bool
            True if the tare values were saved successfully, False otherwise.
        """
        yield from self._imu.tare_accel_gyro(sample_count=sample_count, sample_delay_ms=sample_delay_ms)
        return self._save_tare_only()

    def clear_tare(self):
        """
        Reset the IMU tare offsets to zero and persist the cleared values
        to flash.

        Returns
        -------
        bool
            True if the updated tare values were saved successfully,
            False otherwise.
        """
        self._imu.clear_tare()
        return self._save_tare_only()

    def run(self):
        """
        Cooperative task for scheduler
        """
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
                    self._run_mode_active = False
                    self._state = S5_LOAD_CALIB
                elif mode == 2:
                    self._run_mode_active = False
                    self._state = S4_SAVE_CALIB
                elif mode == 3:
                    self._run_mode_active = False
                    self._state = S6_TARE
                elif mode == 4:
                    self._run_mode_active = False
                    self._state = S7_READ_VALS
                elif mode == 5:
                    self._run_mode_active = False
                    self._state = S8_GET_CALIB_STATE
                elif mode == 0xFF:
                    if not self._run_mode_active:
                        # Tare heading once when switching into run mode.
                        yield from self._imu.tare_euler()
                        self._run_mode_active = True
                    self._state = S3_RUN_NDOF
                else:
                    self._run_mode_active = False


            elif self._state == S3_RUN_NDOF:
                gx, gy, gz = self._imu.gyro()
                h, r, p = self._imu.euler()
                #ax, ay, az = self._imu.acceleration()
                #mx, my, mz = self._imu.magnetic()

                self._heading.put(h)
                self._headingRate.put(gz)

                #self._accelX.put(ax)
                #self._accelY.put(ay)
                #self._accelZ.put(az)
                #self._heading.put(mz)

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
                h, r, p = self._imu.euler()
                #ax, ay, az = self._imu.acceleration()
                #mx, my, mz = self._imu.magnetic()

                self._heading.put(h)
                self._headingRate.put(gz)
                
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
