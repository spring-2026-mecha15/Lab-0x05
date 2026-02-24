from machine import Pin, I2C
import time

# Trigger reset
rst = Pin(Pin.cpu.B15, mode=Pin.OUT_PP)
rst.low()
time.sleep_ms(100)
rst.high()
# Through trial and error, it appears that BNO needs
# > 350ms to boot. Going with 500 to be same
# NOTE: This will need to be a cooperative delay
# in the main program
time.sleep_ms(500)

i2c = I2C(1, freq=100_000)

print("I2C config:", i2c)
print("Devices (7-bit addrs):", [hex(a) for a in i2c.scan()])