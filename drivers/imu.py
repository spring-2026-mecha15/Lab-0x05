from micropython import const
from pyb import I2C
import time
import math


def cooperative_delay_ms(delay_ms, state=0):
    deadline = time.ticks_add(time.ticks_ms(), int(delay_ms))
    while time.ticks_diff(deadline, time.ticks_ms()) > 0:
        yield state


# IDs
BNO055_ID = const(0xA0)             # BNO055 CHIP ID
BNO055_ADDRESS = const(0x28)        # BNO055 I2C address

# Calibration profile length
CALIB_PROFILE_LEN = const(22)


# Registers 
CHIP_ID_REG = const(0x00)           # BNO055 CHIP ID
PAGE_ID_REG = const(0x07)           # Page ID
ACC_DATA_X_LSB_REG = const(0x08)    # Acceleration Data X
MAG_DATA_X_LSB_REG = const(0x0E)    # Magnetometer Data X
GYR_DATA_X_LSB_REG = const(0x14)    # Gyroscope Data X
EUL_HEADING_LSB_REG = const(0x1A)   # Heading Data
QUA_DATA_W_LSB_REG = const(0x20)    # Quaternion Data W
LIA_DATA_X_LSB_REG = const(0x28)    # Linear Acceleration Data X
GRV_DATA_X_LSB_REG = const(0x2E)    # Gravity Vector Data X
TEMP_REG = const(0x34)              # Temperature
CALIB_STAT_REG = const(0x35)        # Calib Status
ACC_OFFSET_X_LSB_REG = const(0x55)  # Accelerometer Offset 
OPR_MODE_REG = const(0x3D)          # Operation Mode
PWR_MODE_REG = const(0x3E)          # Power Mode
SYS_TRIGGER_REG = const(0x3F)       # System Trigger
UNIT_SEL_REG = const(0x3B)          # Unit selection

# Power modes
POWER_MODE_NORMAL = const(0x00)     # Normal Power Mode

# Operation modes
CONFIG_OP_MODE = const(0x00)        # Configuration mode
ACCONLY_OP_MODE = const(0x01)       # Accelerometer only
MAGONLY_OP_MODE = const(0x02)       # Magnetometer only
GYRONLY_OP_MODE = const(0x03)       # Gyroscope only
ACCMAG_OP_MODE = const(0x04)        # Accelerometer + Magnetometer
ACCGYRO_OP_MODE = const(0x05)       # Accelerometer + Gyroscope
MAGGYRO_OP_MODE = const(0x06)       # Magnetometer + Gyroscope
AMG_OP_MODE = const(0x07)           # Accelerometer + Magnetometer + Gyroscope
IMUPLUS_OP_MODE = const(0x08)       # Inertial Measurement Unit
COMPASS_OP_MODE = const(0x09)       # Compass
M4G_OP_MODE = const(0x0A)           # Magnetometer for Gyroscope
NDOF_FMC_OFF_OP_MODE = const(0x0B)  # Nine degrees of freedom with fast magnetic calibration off
NDOF_OP_MODE = const(0x0C)          # Nine degrees of freedom

# Scaling factors (assuming unit selection defaults: m/s^2, rad, rps, Celsius)
SCALE_ACCEL_MS2 = 1.0 / 100.0       # 1 m/s^2 = 100 LSB
SCALE_MAG_UT = 1.0 / 16.0           # 1 uT = 16 LSB
SCALE_GYRO_RPS = (math.pi / 180.0) / 16.0   # rad/s per LSB
SCALE_EULER_RAD = (math.pi / 180.0) / 16.0  # rad per LSB
SCALE_QUAT = 1.0 / 16384.0          # 1 = 2^14 LSB


class BNO055:
    def __init__(self, i2c: I2C, address=BNO055_ADDRESS):
        self.i2c = i2c
        self.address = address
        self.mode = CONFIG_OP_MODE
        self.accel_tare = (0.0, 0.0, 0.0)
        self.gyro_tare = (0.0, 0.0, 0.0)
        self.euler_tare = (0.0, 0.0, 0.0)

    # ---------- Driver Helpers ----------
    def _read_byte(self, register):
        return self.i2c.mem_read(1, self.address, register)[0]

    def _write_byte(self, register, value):
        self.i2c.mem_write(value & 0xFF, self.address, register)

    def _read_bytes(self, register, n):
        return self.i2c.mem_read(n, self.address, register)

    def _write_bytes(self, register, data):
        self.i2c.mem_write(data, self.address, register)

    @staticmethod
    def _bytes_to_int16(lsb, msb):
        v = (msb << 8) | lsb
        return v - 65536 if v & 0x8000 else v

    def _read_vec3_int16(self, base_reg):
        b = self._read_bytes(base_reg, 6)
        x = self._bytes_to_int16(b[0], b[1])
        y = self._bytes_to_int16(b[2], b[3])
        z = self._bytes_to_int16(b[4], b[5])
        return x, y, z


    # ---------- Device Control ----------
    def set_mode(self, mode):
        # per datasheet, switch to CONFIG before changing certain settings
        self._write_byte(OPR_MODE_REG, mode & 0xFF)
        self.mode = mode
        yield from cooperative_delay_ms(30)

    def reset(self):
        # Trigger system reset
        yield from self.set_mode(CONFIG_OP_MODE)
        self._write_byte(SYS_TRIGGER_REG, 0x20)
        yield from cooperative_delay_ms(650)  # typical reset time
        # Confirm sensor functionality after booting
        for _ in range(50):
            if self._read_byte(CHIP_ID_REG) == BNO055_ID:
                return True
            yield from cooperative_delay_ms(10)
        return False

    def initialize(self, mode=NDOF_OP_MODE):
        # Check chip ID
        chip = self._read_byte(CHIP_ID_REG)
        if chip != BNO055_ID:
            # Sometimes needs a moment after power-up
            yield from cooperative_delay_ms(200)
            chip = self._read_byte(CHIP_ID_REG)
            if chip != BNO055_ID:
                raise OSError("BNO055 not found (chip id=0x%02X)" % chip)

        # Config mode
        yield from self.set_mode(CONFIG_OP_MODE)

        # Normal power
        self._write_byte(PWR_MODE_REG, POWER_MODE_NORMAL)
        yield from cooperative_delay_ms(10)

        # Use internal oscillator (common default); leave as-is unless you need external crystal.
        # Clear SYS_TRIGGER clock bits except keep reset bit 0
        self._write_byte(SYS_TRIGGER_REG, 0x00)
        yield from cooperative_delay_ms(10)

        # UNIT_SEL default is fine for this driver (m/s^2, degrees, dps, Celsius).
        # If you changed it elsewhere, set it explicitly here.
        # 0x00 corresponds to:
        #  - Accel m/s^2, Euler degrees, Gyro dps, Temp C, etc. (datasheet unit selection table)
        self._write_byte(UNIT_SEL_REG, 0x00)
        yield from cooperative_delay_ms(10)

        # Switch to requested fusion/non-fusion mode
        yield from self.set_mode(mode)
        yield from cooperative_delay_ms(20)
        return True


    # ---------- Calibration ----------
    def calibration_status(self):
        # 2 bits each: sys, gyro, accel, mag
        calibration = self._read_byte(CALIB_STAT_REG)
        sys = (calibration >> 6) & 0x03
        gyro = (calibration >> 4) & 0x03
        accel = (calibration >> 2) & 0x03
        mag = calibration & 0x03
        return sys, gyro, accel, mag

    def get_calibration_offsets(self):
        prev_mode = self.mode
        yield from self.set_mode(CONFIG_OP_MODE)
        yield from cooperative_delay_ms(25)
        offsets = bytes(self._read_bytes(ACC_OFFSET_X_LSB_REG, CALIB_PROFILE_LEN))
        if prev_mode != CONFIG_OP_MODE:
            yield from self.set_mode(prev_mode)
            yield from cooperative_delay_ms(20)
        return offsets

    def set_calibration_offsets(self, offsets):
        payload = bytes(offsets)
        if len(payload) != CALIB_PROFILE_LEN:
            raise ValueError("offsets must be 22 bytes")

        prev_mode = self.mode
        yield from self.set_mode(CONFIG_OP_MODE)
        yield from cooperative_delay_ms(25)
        self._write_bytes(ACC_OFFSET_X_LSB_REG, payload)
        yield from cooperative_delay_ms(10)
        if prev_mode != CONFIG_OP_MODE:
            yield from self.set_mode(prev_mode)
            yield from cooperative_delay_ms(20)
        return True

    def clear_calibration_offsets(self):
        return (yield from self.set_calibration_offsets(bytes(CALIB_PROFILE_LEN)))


    # ---------- Tare ----------
    def set_tare(self, accel_xyz, gyro_xyz):
        self.accel_tare = (float(accel_xyz[0]), float(accel_xyz[1]), float(accel_xyz[2]))
        self.gyro_tare = (float(gyro_xyz[0]), float(gyro_xyz[1]), float(gyro_xyz[2]))

    def get_tare(self):
        return self.accel_tare, self.gyro_tare

    def set_euler_tare(self, euler_hrp):
        self.euler_tare = (float(euler_hrp[0]), float(euler_hrp[1]), float(euler_hrp[2]))

    def get_euler_tare(self):
        return self.euler_tare

    def clear_tare(self):
        self.accel_tare = (0.0, 0.0, 0.0)
        self.gyro_tare = (0.0, 0.0, 0.0)
        self.euler_tare = (0.0, 0.0, 0.0)

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

        self.accel_tare = (
            sum_ax / sample_count,
            sum_ay / sample_count,
            sum_az / sample_count,
        )
        self.gyro_tare = (
            sum_gx / sample_count,
            sum_gy / sample_count,
            sum_gz / sample_count,
        )

        return self.accel_tare, self.gyro_tare

    def tare_euler(self, sample_count=20, sample_delay_ms=5):
        if sample_count <= 0:
            raise ValueError("sample_count must be > 0")

        sum_h = 0.0
        sum_r = 0.0
        sum_p = 0.0

        for _ in range(sample_count):
            h, r, p = self.euler_raw()
            sum_h += h
            sum_r += r
            sum_p += p
            if sample_delay_ms > 0:
                yield from cooperative_delay_ms(sample_delay_ms)

        self.euler_tare = (
            sum_h / sample_count,
            sum_r / sample_count,
            sum_p / sample_count,
        )

        return self.euler_tare


    # ---------- Sensor Readings  ----------
    def temperature(self):
        temp = self._read_byte(TEMP_REG)
        # signed 8-bit
        return temp - 256 if temp & 0x80 else temp

    def acceleration_raw(self):
        x, y, z = self._read_vec3_int16(ACC_DATA_X_LSB_REG)
        return (x * SCALE_ACCEL_MS2, y * SCALE_ACCEL_MS2, z * SCALE_ACCEL_MS2)

    def gyro_raw(self):
        x, y, z = self._read_vec3_int16(GYR_DATA_X_LSB_REG)
        return (x * SCALE_GYRO_RPS, y * SCALE_GYRO_RPS, z * SCALE_GYRO_RPS)

    def euler_raw(self):
        # heading, roll, pitch
        h, r, p = self._read_vec3_int16(EUL_HEADING_LSB_REG)
        return (h * SCALE_EULER_RAD, r * SCALE_EULER_RAD, p * SCALE_EULER_RAD)

    def acceleration(self):
        ax, ay, az = self.acceleration_raw()
        tx, ty, tz = self.accel_tare
        return (ax - tx, ay - ty, az - tz)

    def magnetic(self):
        x, y, z = self._read_vec3_int16(MAG_DATA_X_LSB_REG)
        return (x * SCALE_MAG_UT, y * SCALE_MAG_UT, z * SCALE_MAG_UT)

    def gyro(self):
        gx, gy, gz = self.gyro_raw()
        tx, ty, tz = self.gyro_tare
        return (gx - tx, gy - ty, gz - tz)

    def euler(self):
        h, r, p = self.euler_raw()
        th, tr, tp = self.euler_tare
        return (h - th, r - tr, p - tp)

    def quaternion(self):
        b = self._read_bytes(QUA_DATA_W_LSB_REG, 8)
        w = self._bytes_to_int16(b[0], b[1]) * SCALE_QUAT
        x = self._bytes_to_int16(b[2], b[3]) * SCALE_QUAT
        y = self._bytes_to_int16(b[4], b[5]) * SCALE_QUAT
        z = self._bytes_to_int16(b[6], b[7]) * SCALE_QUAT
        return (w, x, y, z)

    def linear_acceleration(self):
        x, y, z = self._read_vec3_int16(LIA_DATA_X_LSB_REG)
        return (x * SCALE_ACCEL_MS2, y * SCALE_ACCEL_MS2, z * SCALE_ACCEL_MS2)

    def gravity(self):
        x, y, z = self._read_vec3_int16(GRV_DATA_X_LSB_REG)
        return (x * SCALE_ACCEL_MS2, y * SCALE_ACCEL_MS2, z * SCALE_ACCEL_MS2)
