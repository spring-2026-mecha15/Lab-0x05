"""
User interface package for the Romi robot line-follower.

Provides interactive serial-terminal screens for configuring and monitoring
the robot at runtime. Each screen is a generator-based coroutine that yields
control to the cooperative scheduler, enabling non-blocking interaction while
other tasks (motor control, sensing) continue to run.

Screens included:
    - ``ui_calibration``: Two-phase reflectance sensor calibration (light/dark)
    - ``ui_debug``:        Real-time sensor readout and battery voltage display
    - ``ui_gains``:        Interactive PID and feed-forward gain tuning wizard
    - ``ui_imu``:          IMU submenu for calibration load/save, tare, and status
    - ``ui_linefollow``:   Activate autonomous line-following and stream telemetry
    - ``ui_setpoint``:     Adjust motor speed and line-follower setpoints
"""
