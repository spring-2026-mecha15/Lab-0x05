from pyb import I2C
import time
from micropython import const


BOARD_ADDRESS = const(0x28)

OPERATION_MODE_CONFIG = const(0x08)
OPERATION_MODE_CONFIG = const(0x08)


class IMU:
    """
    SCL: grey ->
    SDA: purple ->
    RST: white ->

    """

    def __init__(self, i2c):
        self._i2c_channel = i2c

    def cooperative_delay(self, delay_ms, state=0):
        deadline = time.ticks_add(time.ticks_ms(), int(delay_ms))
        while time.ticks_diff(deadline, time.ticks_ms()) > 0:
            yield state

    def _eight_bit_write(self, value, register, wait_time=None):
        self._i2c_channel.mem_write(value, BOARD_ADDRESS, register)
        if wait_time is not None:
            yield from self.cooperative_delay(wait_time)

    def change_operating_mode(self):
        yield from self._eight_bit_write(0x00, 0x3D, 19)
        yield from self._eight_bit_write(0x08, 0x3D, 7)
        return
        
    def retrieve_calibration_state(self):
        # 2 bits each: sys, gyro, accel, mag
        status = self._i2c_channel.mem_read(1, self._adr, 0x35)[0]
        sys = (status >> 6) & 0x03
        gyro = (status >> 4) & 0x03
        accel = (status >> 2) & 0x03
        mag = status & 0x03
        return sys, gyro, accel, mag


    def write_calibration_coefficients(self):
        # Blank Template.
        print()

    def read_Euler_angles(self):
        # Blank Template.
        print()

    def calibrate(self): 
        # Blank Template.
        print()


#Keep going though and filling out all of the functions