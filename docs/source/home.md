# Home

**Authors:** Max Schecter & Gabe Haarberg

**Course:** ME 405 — Mechatronics, Cal Poly SLO

**Platform:** Pololu Romi chassis · STM32 Nucleo · MicroPython

---

## Overview

Romi is an autonomous line-following robot built on the Pololu Romi chassis. It was built for a competition in the Cal Poly ME 405 class. It uses a 7-element IR reflectance array, a BNO055 IMU, quadrature encoders, and a model-based state observer to navigate a competition course reliably and at speed.

## Robot Photo

```{image} _static/romi_closeup.png
:alt: Romi robot
:width: 600px
:align: center
```

---

## Video Demonstration

<iframe width="700" height="394"
  src="https://www.youtube.com/embed/ol2RzCHO2P4"
  frameborder="0" allowfullscreen>
</iframe>

---

## Key Features

- 7-element IR reflectance array (Pololu QTRX) for line sensing and weighted-centroid calculation
- BNO055 9-DOF IMU for heading and heading-rate feedback over I2C
- Quadrature encoders on both wheels (1437.12 counts/rev, 35 mm wheel radius) for odometry and velocity feedback
- PI velocity controllers on each motor (Kp = 0.15, Ki = 4.0) with conditional anti-windup
- Proportional + integral + feed-forward line-following controller (Kp = 0.40, Ki = 0.30, Kff = 0–0.6 per segment)
- 9-task cooperative scheduler (cotask) — tasks communicate via 41 shared variables and 4 data queues
- 15-state autonomous competition task (task_competition) managing acceleration ramps, arc turns, garage entry, and U-turn sequencing
- Discrete-time Luenberger observer designed for the linearized model (wheel-average fallback used in competition for reliability)

---

## Quick Links

- [Hardware Design](hardware.md) — sensors, wiring, and mechanical design
- [Software Architecture](software.md) — task diagram, control scheme, and algorithms
- [FSM Documentation](FSMs.md) — state machine diagrams for every task
- [Results & Reflection](results.md) — time trial performance and lessons learned
- [API Reference](api/generated/modules.rst) — full auto-generated code documentation
