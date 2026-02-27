# from machine import Pin, I2C
# import time

# def cooperative_delay_ms(delay_ms, state=0):
#     deadline = time.ticks_add(time.ticks_ms(), int(delay_ms))
#     while time.ticks_diff(deadline, time.ticks_ms()) > 0:
#         yield state


# def reset_sequence(state=0):
#     rst = Pin(Pin.cpu.B15, mode=Pin.OUT_PP)
#     rst.low()
#     yield from cooperative_delay_ms(100, state)
#     rst.high()
#     yield from cooperative_delay_ms(500, state)


# def create_default_i2c(freq=100_000):
#     return I2C(1, freq=freq)


###########################################################
###########################################################

# Below is an adaptation by ChatGPT of the library from
# https://github.com/adafruit/Adafruit_BNO055/tree/master

###########################################################
###########################################################


# bno055.py
# Minimal MicroPython driver for Bosch BNO055 (I2C)
#
# Scales based on Bosch BNO055 datasheet:
# - Euler: 1 degree = 16 LSB
# - Quaternion: 1 = 2^14 LSB
# - Accel (m/s^2): 1 m/s^2 = 100 LSB
# - Mag (uT): 1 uT = 16 LSB
# - Gyro (dps): 1 dps = 16 LSB
#
# See citations in assistant message.

from micropython import const
from pyb import I2C
import time


def cooperative_delay_ms(delay_ms, state=0):
    deadline = time.ticks_add(time.ticks_ms(), int(delay_ms))
    while time.ticks_diff(deadline, time.ticks_ms()) > 0:
        yield state

# I2C addresses (ADR low/high)
BNO055_ADDRESS_A = const(0x28)
BNO055_ADDRESS_B = const(0x29)

# IDs
BNO055_ID = const(0xA0)

# Registers (page 0 unless noted)
_CHIP_ID = const(0x00)
_PAGE_ID = const(0x07)

_ACC_DATA_X_LSB = const(0x08)
_MAG_DATA_X_LSB = const(0x0E)
_GYR_DATA_X_LSB = const(0x14)
_EUL_HEADING_LSB = const(0x1A)
_QUA_DATA_W_LSB = const(0x20)
_LIA_DATA_X_LSB = const(0x28)
_GRV_DATA_X_LSB = const(0x2E)
_TEMP = const(0x34)

_CALIB_STAT = const(0x35)

_ACC_OFFSET_X_LSB = const(0x55)
_CALIB_PROFILE_LEN = const(22)

_OPR_MODE = const(0x3D)
_PWR_MODE = const(0x3E)
_SYS_TRIGGER = const(0x3F)
_UNIT_SEL = const(0x3B)

# Power modes
POWER_MODE_NORMAL = const(0x00)

# Operation modes (Adafruit values)
OPERATION_MODE_CONFIG = const(0x00)
OPERATION_MODE_ACCONLY = const(0x01)
OPERATION_MODE_MAGONLY = const(0x02)
OPERATION_MODE_GYRONLY = const(0x03)
OPERATION_MODE_ACCMAG = const(0x04)
OPERATION_MODE_ACCGYRO = const(0x05)
OPERATION_MODE_MAGGYRO = const(0x06)
OPERATION_MODE_AMG = const(0x07)
OPERATION_MODE_IMUPLUS = const(0x08)
OPERATION_MODE_COMPASS = const(0x09)
OPERATION_MODE_M4G = const(0x0A)
OPERATION_MODE_NDOF_FMC_OFF = const(0x0B)
OPERATION_MODE_NDOF = const(0x0C)

# Datasheet scaling factors (assuming UNIT_SEL defaults: m/s^2, degrees, dps, Celsius)
_SCALE_ACCEL_MS2 = 1.0 / 100.0     # 1 m/s^2 = 100 LSB
_SCALE_MAG_UT = 1.0 / 16.0         # 1 uT = 16 LSB
_SCALE_GYRO_DPS = 1.0 / 16.0       # 1 dps = 16 LSB
_SCALE_EULER_DEG = 1.0 / 16.0      # 1 degree = 16 LSB
_SCALE_QUAT = 1.0 / 16384.0        # 1 = 2^14 LSB


class BNO055:
    def __init__(self, i2c: I2C, address=BNO055_ADDRESS_A):
        self.i2c = i2c
        self.address = address
        self._mode = OPERATION_MODE_CONFIG
        self._accel_tare = (0.0, 0.0, 0.0)
        self._gyro_tare = (0.0, 0.0, 0.0)

    # ---------- low-level I2C helpers ----------
    def _read_u8(self, reg):
        return self.i2c.mem_read(1, self.address, reg)[0]

    def _write_u8(self, reg, val):
        self.i2c.mem_write(val & 0xFF, self.address, reg)

    def _read_len(self, reg, n):
        return self.i2c.mem_read(n, self.address, reg)

    def _write_len(self, reg, data):
        self.i2c.mem_write(data, self.address, reg)

    @staticmethod
    def _to_int16(lsb, msb):
        v = (msb << 8) | lsb
        return v - 65536 if v & 0x8000 else v

    def _read_vec3_i16(self, base_reg):
        b = self._read_len(base_reg, 6)
        x = self._to_int16(b[0], b[1])
        y = self._to_int16(b[2], b[3])
        z = self._to_int16(b[4], b[5])
        return x, y, z

    # ---------- device control ----------
    def set_mode(self, mode):
        # per datasheet, switch to CONFIG before changing certain settings
        self._write_u8(_OPR_MODE, mode & 0xFF)
        self._mode = mode
        yield from cooperative_delay_ms(30)

    def reset(self):
        # Trigger system reset
        yield from self.set_mode(OPERATION_MODE_CONFIG)
        self._write_u8(_SYS_TRIGGER, 0x20)
        yield from cooperative_delay_ms(650)  # typical reset time
        # After reset, chip ID may read 0x00 briefly
        for _ in range(50):
            if self._read_u8(_CHIP_ID) == BNO055_ID:
                return True
            yield from cooperative_delay_ms(10)
        return False

    def begin(self, mode=OPERATION_MODE_NDOF):
        # Check chip ID
        chip = self._read_u8(_CHIP_ID)
        if chip != BNO055_ID:
            # Sometimes needs a moment after power-up
            yield from cooperative_delay_ms(200)
            chip = self._read_u8(_CHIP_ID)
            if chip != BNO055_ID:
                raise OSError("BNO055 not found (chip id=0x%02X)" % chip)

        # Config mode
        yield from self.set_mode(OPERATION_MODE_CONFIG)

        # Normal power
        self._write_u8(_PWR_MODE, POWER_MODE_NORMAL)
        yield from cooperative_delay_ms(10)

        # Use internal oscillator (common default); leave as-is unless you need external crystal.
        # Clear SYS_TRIGGER clock bits except keep reset bit 0
        self._write_u8(_SYS_TRIGGER, 0x00)
        yield from cooperative_delay_ms(10)

        # UNIT_SEL default is fine for this driver (m/s^2, degrees, dps, Celsius).
        # If you changed it elsewhere, set it explicitly here.
        # 0x00 corresponds to:
        #  - Accel m/s^2, Euler degrees, Gyro dps, Temp C, etc. (datasheet unit selection table)
        self._write_u8(_UNIT_SEL, 0x00)
        yield from cooperative_delay_ms(10)

        # Switch to requested fusion/non-fusion mode
        yield from self.set_mode(mode)
        yield from cooperative_delay_ms(20)
        return True

    # ---------- high-level reads (scaled) ----------
    def temperature_c(self):
        t = self._read_u8(_TEMP)
        # signed 8-bit
        return t - 256 if t & 0x80 else t

    def calibration_status(self):
        # 2 bits each: sys, gyro, accel, mag
        c = self._read_u8(_CALIB_STAT)
        sys = (c >> 6) & 0x03
        gyro = (c >> 4) & 0x03
        accel = (c >> 2) & 0x03
        mag = c & 0x03
        return sys, gyro, accel, mag

    def get_calibration_offsets(self):
        prev_mode = self._mode
        yield from self.set_mode(OPERATION_MODE_CONFIG)
        yield from cooperative_delay_ms(25)
        offsets = bytes(self._read_len(_ACC_OFFSET_X_LSB, _CALIB_PROFILE_LEN))
        if prev_mode != OPERATION_MODE_CONFIG:
            yield from self.set_mode(prev_mode)
            yield from cooperative_delay_ms(20)
        return offsets

    def set_calibration_offsets(self, offsets):
        payload = bytes(offsets)
        if len(payload) != _CALIB_PROFILE_LEN:
            raise ValueError("offsets must be 22 bytes")

        prev_mode = self._mode
        yield from self.set_mode(OPERATION_MODE_CONFIG)
        yield from cooperative_delay_ms(25)
        self._write_len(_ACC_OFFSET_X_LSB, payload)
        yield from cooperative_delay_ms(10)
        if prev_mode != OPERATION_MODE_CONFIG:
            yield from self.set_mode(prev_mode)
            yield from cooperative_delay_ms(20)
        return True

    def clear_calibration_offsets(self):
        return (yield from self.set_calibration_offsets(bytes(_CALIB_PROFILE_LEN)))

    def set_tare(self, accel_xyz, gyro_xyz):
        self._accel_tare = (float(accel_xyz[0]), float(accel_xyz[1]), float(accel_xyz[2]))
        self._gyro_tare = (float(gyro_xyz[0]), float(gyro_xyz[1]), float(gyro_xyz[2]))

    def get_tare(self):
        return self._accel_tare, self._gyro_tare

    def clear_tare(self):
        self._accel_tare = (0.0, 0.0, 0.0)
        self._gyro_tare = (0.0, 0.0, 0.0)

    def tare_accel_gyro(self, sample_count=100, sample_delay_ms=5):
        if sample_count <= 0:
            raise ValueError("sample_count must be > 0")

        sum_ax = 0.0
        sum_ay = 0.0
        sum_az = 0.0
        sum_gx = 0.0
        sum_gy = 0.0
        sum_gz = 0.0

        for _ in range(sample_count):
            ax, ay, az = self.acceleration_raw()
            gx, gy, gz = self.gyro_raw()
            sum_ax += ax
            sum_ay += ay
            sum_az += az
            sum_gx += gx
            sum_gy += gy
            sum_gz += gz
            if sample_delay_ms > 0:
                yield from cooperative_delay_ms(sample_delay_ms)

        self._accel_tare = (
            sum_ax / sample_count,
            sum_ay / sample_count,
            sum_az / sample_count,
        )
        self._gyro_tare = (
            sum_gx / sample_count,
            sum_gy / sample_count,
            sum_gz / sample_count,
        )

        return self._accel_tare, self._gyro_tare

    def acceleration_raw(self):
        x, y, z = self._read_vec3_i16(_ACC_DATA_X_LSB)
        return (x * _SCALE_ACCEL_MS2, y * _SCALE_ACCEL_MS2, z * _SCALE_ACCEL_MS2)

    def gyro_raw(self):
        x, y, z = self._read_vec3_i16(_GYR_DATA_X_LSB)
        return (x * _SCALE_GYRO_DPS, y * _SCALE_GYRO_DPS, z * _SCALE_GYRO_DPS)

    def acceleration(self):
        ax, ay, az = self.acceleration_raw()
        tx, ty, tz = self._accel_tare
        return (ax - tx, ay - ty, az - tz)

    def magnetic(self):
        x, y, z = self._read_vec3_i16(_MAG_DATA_X_LSB)
        return (x * _SCALE_MAG_UT, y * _SCALE_MAG_UT, z * _SCALE_MAG_UT)

    def gyro(self):
        gx, gy, gz = self.gyro_raw()
        tx, ty, tz = self._gyro_tare
        return (gx - tx, gy - ty, gz - tz)

    def euler(self):
        # heading, roll, pitch
        h, r, p = self._read_vec3_i16(_EUL_HEADING_LSB)
        return (h * _SCALE_EULER_DEG, r * _SCALE_EULER_DEG, p * _SCALE_EULER_DEG)

    def quaternion(self):
        b = self._read_len(_QUA_DATA_W_LSB, 8)
        w = self._to_int16(b[0], b[1]) * _SCALE_QUAT
        x = self._to_int16(b[2], b[3]) * _SCALE_QUAT
        y = self._to_int16(b[4], b[5]) * _SCALE_QUAT
        z = self._to_int16(b[6], b[7]) * _SCALE_QUAT
        return (w, x, y, z)

    def linear_acceleration(self):
        x, y, z = self._read_vec3_i16(_LIA_DATA_X_LSB)
        return (x * _SCALE_ACCEL_MS2, y * _SCALE_ACCEL_MS2, z * _SCALE_ACCEL_MS2)

    def gravity(self):
        x, y, z = self._read_vec3_i16(_GRV_DATA_X_LSB)
        return (x * _SCALE_ACCEL_MS2, y * _SCALE_ACCEL_MS2, z * _SCALE_ACCEL_MS2)