# Roami

**An autonomous line-following robot built on the Pololu Romi platform for ME 405 Mechatronics, Cal Poly SLO — Winter 2026.**

**Authors:** Max Schecter & Gabe Haarberg

---

## Overview

Roami is an autonomous differential-drive robot that navigates a competition course using closed-loop line following, IMU-assisted heading control, and ultrasonic obstacle detection. The robot runs entirely on an STM32 Nucleo microcontroller programmed in MicroPython, coordinating nine concurrent tasks through a cooperative scheduler. Across three competition runs, Roami completed the course consistently under 55.7 seconds with a run-to-run spread of only 0.2 seconds.

Full documentation, hardware schematics, controller derivations, state machine diagrams, and competition results are available at the project website:
**[https://spring-2026-mecha15.github.io/Romi-Project/](https://spring-2026-mecha15.github.io/Romi-Project/)**

---

## Hardware

- **Chassis:** Pololu Romi differential-drive platform
- **Microcontroller:** STM32 Nucleo (MicroPython)
- **Line sensor:** Pololu QTRX 7-element IR reflectance array
- **IMU:** BNO055 9-DOF (accelerometer, gyroscope, magnetometer)
- **Encoders:** Quadrature encoders on each drive wheel
- **Proximity:** HC-SR04 ultrasonic distance sensor

---

## Software

- **Scheduler:** 9-task cooperative multitasking via `cotask`
- **Velocity control:** Independent PI controllers for left and right wheels with encoder feedback
- **Line following:** PI + feed-forward controller driven by the IR reflectance centroid
- **State machine:** 15-state finite state machine governing the full competition run
- **State estimation:** Luenberger observer design (simplified to wheel-average for competition)
- **Inter-task communication:** Lock-free shared variables and queues via `task_share`

---

## Repository Structure

```
Roami/
├── drivers/
│   ├── motor.py          # Motor driver (PWM + direction + battery compensation)
│   ├── encoder.py        # Quadrature encoder reader
│   ├── imu.py            # BNO055 IMU driver
│   ├── reflectance.py    # QTRX IR array driver
│   └── ultrasonic.py     # HC-SR04 ultrasonic driver
├── ui/
│   ├── ui_debug.py       # Debug submenu
│   ├── ui_calibration.py # Reflectance sensor calibration submenu
│   ├── ui_gains.py       # PI gain tuning submenu
│   ├── ui_setpoint.py    # Speed setpoint submenu
│   ├── ui_imu.py         # BNO055 IMU calibration submenu
│   └── ui_linefollow.py  # Line-follow mode submenu
├── task_*.py             # Individual cooperative tasks (line follow, motor, FSM, etc.)
├── main.py               # Entry point — instantiates tasks and starts the scheduler
├── controller.py         # PI and feed-forward controller implementations
├── cotask.py             # Cooperative task scheduler
├── task_share.py         # Inter-task shared variable and queue primitives
├── multichar_input.py    # Multi-character serial input utility
├── constants.py          # Tuned gains and system-wide constants
├── desktop/
│   ├── host.py           # Host-side serial data receiver
│   ├── plot.py           # Post-run data visualization
│   ├── UI_Functions.py   # Desktop UI utilities
│   └── requirements.txt  # Desktop Python dependencies
└── docs/                 # Sphinx documentation source
```

---

## Documentation

[https://spring-2026-mecha15.github.io/Romi-Project/](https://spring-2026-mecha15.github.io/Romi-Project/)
