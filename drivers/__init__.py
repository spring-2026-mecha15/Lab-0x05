"""
Hardware driver package for the Romi robot platform.

Provides device-level abstractions for each hardware peripheral on the Romi,
all targeting MicroPython on the STM32 microcontroller.

Drivers included:
    - ``encoder``:      Quadrature encoder reader for wheel odometry and velocity feedback
    - ``imu``:          BNO055 IMU driver (accelerometer, gyroscope, magnetometer, heading)
    - ``motor``:        PWM motor driver with direction and enable control
    - ``reflectance``:  IR reflectance sensor array for line detection and centroid calculation
    - ``ultrasonic``:   HC-SR04 ultrasonic distance sensor for obstacle detection
"""
