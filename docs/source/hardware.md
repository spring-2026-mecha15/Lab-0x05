# Hardware Design

## Platform

| Component | Part |
|-----------|------|
| Chassis | Pololu Romi 32U4 |
| Microcontroller | STM32 Nucleo-L476RG |
| Motor driver | On-board Romi DRV8838 (via Nucleo GPIO) |
| Encoders | Pololu magnetic quadrature encoders (1437.12 counts/rev) |
| Line sensor | Pololu QTRX-MD-07A (7-element IR reflectance array) |
| IMU | Bosch BNO055 (I2C, 100 kHz) |
| Battery | 6× AA (nominally ~9 V) |

---

## Wiring Diagram

<!-- TODO: insert wiring diagram image -->
<!-- ![Wiring Diagram](../_static/wiring_diagram.png) -->

*A wiring diagram will be added here. See the repository for the source file.*

### Pin Assignments

| Signal | STM32 Pin | Notes |
|--------|-----------|-------|
| Right motor PWM | PA6 | Timer 3, CH1, 50 kHz |
| Left motor PWM | PA7 | Timer 3, CH2, 50 kHz |
| Right motor DIR | PC7 | GPIO output |
| Right motor SLP | PB6 | GPIO output (active-high enable) |
| Left motor DIR | PB11 | GPIO output |
| Left motor SLP | PB12 | GPIO output |
| Right encoder A | PA8 | Timer 1, CH1 |
| Right encoder B | PA9 | Timer 1, CH2 |
| Left encoder A | PA1 | Timer 2, CH2 |
| Left encoder B | PA0 | Timer 2, CH1 |
| QTRX sensors | PA4, PA5, PB0, PC0–PC3 | ADC inputs |
| IMU SDA | PB13 | I2C1 (AF-4) |
| IMU SCL | PB14 | I2C1 (AF-4) |
| IMU RST | PB15 | GPIO output |
| Battery ADC | PB1 | Voltage monitoring |

---

## Mechanical Design

<!-- TODO: describe sensor mount, chassis modifications, etc. -->
<!-- TODO: add photos -->

### Sensor Mount
<!-- Describe how the QTRX sensor is mounted to the chassis -->

### IMU Placement
<!-- Describe IMU orientation and mounting location -->

---

## Photos

<!-- TODO: add photos of the assembled robot -->
<!-- ![Robot Photo 1](../_static/robot_front.jpg) -->
<!-- ![Robot Photo 2](../_static/robot_top.jpg) -->
