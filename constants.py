try:
    from pyb import Pin

    MOTOR_R_SLP = Pin.cpu.B6
    MOTOR_R_DIR = Pin.cpu.C7
    MOTOR_R_PWM = Pin.cpu.A6

    MOTOR_L_SLP = Pin.cpu.B12
    MOTOR_L_DIR = Pin.cpu.B11
    MOTOR_L_PWM = Pin.cpu.A7

    ENCODER_R_A = Pin.cpu.A8
    ENCODER_R_B = Pin.cpu.A9

    ENCODER_L_A = Pin.cpu.A1
    ENCODER_L_B = Pin.cpu.A0

    # IMU connected to I2C Bus 2
    # Pins must be configured to use AF-4
    # https://www.st.com/resource/en/datasheet/stm32l476rg.pdf
    # Table-17, page 89.
    IMU_RST = Pin.cpu.B15
    IMU_SCL = Pin.cpu.B14
    IMU_SDA = Pin.cpu.B13

    QTRX_A1 = Pin.cpu.A5
    QTRX_A3 = Pin.cpu.C2
    QTRX_A5 = Pin.cpu.C3
    QTRX_A7 = Pin.cpu.A4
    QTRX_A9 = Pin.cpu.B0
    QTRX_A11 = Pin.cpu.C1
    QTRX_A13 = Pin.cpu.C0

    BATT_ADC = Pin.cpu.B1

except ImportError:
    print('Info: skipping pin definitions on non-STM32 device')

PI = 3.1415926536

ECOUNTS_PER_WREV   = 1437.12    # Number of encoder counts
                                # per wheel revolution

WHEEL_RADIUS       = 35         # Wheel radius in mm

ENCODER_GAIN       = 2 * PI * WHEEL_RADIUS * 1000 / ECOUNTS_PER_WREV
                                # Encoder gain for PI
                                # controller class

MOTOR_CONST        = 4.5 / 150  # RPM/V from pololu specs


MOTOR_GAIN         = MOTOR_CONST * 60 * 100 / (2 * PI * WHEEL_RADIUS)
                                # Motor gain for PI
                                # controller class

CSV_BEGIN = '>>>>CSV START<<<<'            
CSV_END = '>>>>CSV END<<<<'            

GAINS_FILE = "gains.json"

# # Typical main.py setup:
# from pyb import Timer
# from drivers.motor import Motor
# from drivers.encoder import Encoder
# from constants import *

# tim1 = Timer(1) # For R encoder
# tim2 = Timer(2) # For L encoder
# tim3 = Timer(3, freq=50000) # For motor PWMs

# motor_right = Motor(MOTOR_R_PWM, tim3, 1, MOTOR_R_SLP, MOTOR_R_DIR)
# motor_left = Motor(MOTOR_L_PWM, tim3, 2, MOTOR_L_SLP, MOTOR_L_DIR)
# encoder_r = Encoder(tim1, ENCODER_R_A, ENCODER_R_B)
# encoder_l = Encoder(tim2, ENCODER_L_A, ENCODER_L_B)
