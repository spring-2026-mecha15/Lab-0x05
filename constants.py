"""
constants.py

Hardware pin assignments, physical parameters, control gains, and
file-path constants shared across all tasks in the Romi robot project.

Pin constants are defined only when running on the STM32 target (pyb
available); they are skipped silently on desktop/CPython.
"""

try:
    from pyb import Pin

    #: Right motor sleep (enable) pin — STM32 CPU pin B6.
    MOTOR_R_SLP = Pin.cpu.B6
    #: Right motor direction pin — STM32 CPU pin C7.
    MOTOR_R_DIR = Pin.cpu.C7
    #: Right motor PWM output pin — STM32 CPU pin A6 (Timer 3 CH1).
    MOTOR_R_PWM = Pin.cpu.A6

    #: Left motor sleep (enable) pin — STM32 CPU pin B12.
    MOTOR_L_SLP = Pin.cpu.B12
    #: Left motor direction pin — STM32 CPU pin B11.
    MOTOR_L_DIR = Pin.cpu.B11
    #: Left motor PWM output pin — STM32 CPU pin A7 (Timer 3 CH2).
    MOTOR_L_PWM = Pin.cpu.A7

    #: Right encoder channel A input — STM32 CPU pin A8 (Timer 1 CH1).
    ENCODER_R_A = Pin.cpu.A8
    #: Right encoder channel B input — STM32 CPU pin A9 (Timer 1 CH2).
    ENCODER_R_B = Pin.cpu.A9

    #: Left encoder channel A input — STM32 CPU pin A1 (Timer 2 CH2).
    ENCODER_L_A = Pin.cpu.A1
    #: Left encoder channel B input — STM32 CPU pin A0 (Timer 2 CH1).
    ENCODER_L_B = Pin.cpu.A0

    # IMU connected to I2C Bus 2.
    # Pins must be configured to use AF-4.
    # See STM32L476RG datasheet Table-17, page 89.
    #: BNO055 IMU reset pin — STM32 CPU pin B15.
    IMU_RST = Pin.cpu.B15
    #: BNO055 IMU I2C clock pin — STM32 CPU pin B14 (I2C2 SCL, AF-4).
    IMU_SCL = Pin.cpu.B14
    #: BNO055 IMU I2C data pin — STM32 CPU pin B13 (I2C2 SDA, AF-4).
    IMU_SDA = Pin.cpu.B13

    #: QTRX reflectance sensor array pin 1 (leftmost) — STM32 CPU pin A5.
    QTRX_A1  = Pin.cpu.A5
    #: QTRX reflectance sensor array pin 3 — STM32 CPU pin C2.
    QTRX_A3  = Pin.cpu.C2
    #: QTRX reflectance sensor array pin 5 — STM32 CPU pin C3.
    QTRX_A5  = Pin.cpu.C3
    #: QTRX reflectance sensor array pin 7 (center) — STM32 CPU pin A4.
    QTRX_A7  = Pin.cpu.A4
    #: QTRX reflectance sensor array pin 9 — STM32 CPU pin B0.
    QTRX_A9  = Pin.cpu.B0
    #: QTRX reflectance sensor array pin 11 — STM32 CPU pin C1.
    QTRX_A11 = Pin.cpu.C1
    #: QTRX reflectance sensor array pin 13 (rightmost) — STM32 CPU pin C0.
    QTRX_A13 = Pin.cpu.C0

    #: Ultrasonic sensor trigger output pin — STM32 CPU pin B3.
    PIN_TRIG = Pin.cpu.B3
    #: Ultrasonic sensor echo input pin — STM32 CPU pin B4.
    PIN_ECHO = Pin.cpu.B4

    #: Battery voltage ADC input pin — STM32 CPU pin B1.
    BATT_ADC = Pin.cpu.B1

except ImportError:
    # Pin definitions are skipped on non-STM32 (desktop/CPython) targets.
    pass

#: Pi to 10 significant figures.
PI = 3.1415926536

#: Number of encoder counts per full wheel revolution (quadrature, Romi gearbox).
ECOUNTS_PER_WREV = 1437.12

#: Wheel radius in millimeters.
WHEEL_RADIUS = 35

#: Encoder linear gain: converts encoder counts to millimeters of wheel travel.
#: Derived as ``2 * pi * WHEEL_RADIUS / ECOUNTS_PER_WREV`` (mm/count).
ENCODER_GAIN = 2 * PI * WHEEL_RADIUS * 1000 / ECOUNTS_PER_WREV

#: Motor speed constant in RPM/V (from Pololu specifications, 150 RPM at 4.5 V).
MOTOR_CONST = 4.5 / 150

#: Motor linear gain: converts motor voltage (V) to wheel linear velocity (mm/s).
#: Derived as ``MOTOR_CONST * 60 / (2 * pi * WHEEL_RADIUS) * 100``.
MOTOR_GAIN = MOTOR_CONST * 60 * 100 / (2 * PI * WHEEL_RADIUS)

#: Sentinel string printed before CSV data begins over serial.
CSV_BEGIN = '>>>>CSV START<<<<'

#: Sentinel string printed after CSV data ends over serial.
CSV_END = '>>>>CSV END<<<<'

#: Filename for the JSON file storing PI controller gains on the MCU filesystem.
GAINS_FILE = "gains.json"

#: Filename for the JSON file storing BNO055 IMU calibration offsets and tare values.
IMU_FILE = "imu.json"

#: Default proportional gain for motor velocity PI controllers.
DEFAULT_MOTOR_KP = 0.15

#: Default integral gain for motor velocity PI controllers.
DEFAULT_MOTOR_KI = 4.0

#: Default proportional gain for the line-follow PI controller.
DEFAULT_LF_KP = 0.40

#: Default integral gain for the line-follow PI controller.
DEFAULT_LF_KI = 0.30

#: Default feed-forward gain for the line-follow controller.
DEFAULT_LF_KFF = 0.5
