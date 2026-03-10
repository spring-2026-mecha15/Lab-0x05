# Project Overview

## What It Does

Mecha15 is an autonomous mobile robot that follows a painted line course at variable
speed. The robot navigates a multi-leg competition track that includes:

- A straight high-speed acceleration/deceleration run
- A 200 mm-radius 180° curve
- A section with an intermittent (dashed) line
- A "square wave" / offset lane section
- *(Planned)* A parking garage maneuver

Control is closed-loop at every level: individual motor speeds are regulated by PI
controllers, line centering is handled by a second PI loop, and a discrete-time
Luenberger observer fuses encoder and IMU data to provide a global pose estimate that
drives the high-level competition sequencer.

---

## System Architecture

The software is structured as a set of cooperatively scheduled tasks running on
MicroPython. Inter-task communication uses lock-free `Share` (single value) and
`Queue` (FIFO buffer) objects from `task_share.py`.

<!-- TODO: insert task diagram image here -->
<!-- ![Task Diagram](../_static/task_diagram.png) -->

| Task | Period | Priority | Role |
|------|--------|----------|------|
| Competition | 50 ms | 7 (highest) | High-level course sequencer FSM |
| Observer | 20 ms | 6 | State estimation (pose, velocity) |
| IMU | 50 ms | 5 | BNO055 heading + yaw rate |
| Right Motor | 20 ms | 4 | PI speed control — right wheel |
| Left Motor | 20 ms | 3 | PI speed control — left wheel |
| Reflectance | 50 ms | 2 | 7-sensor IR array → line centroid |
| Line Follow | 40 ms | 1 | PI line-centering → motor setpoints |
| User Interface | aperiodic | 0 (lowest) | Serial tuning & data export |
| Garbage Collect | 5000 ms | 0 | MicroPython memory management |

Higher priority numbers run first in the `cotask` priority scheduler.

---

## Key Design Decisions

### Cooperative Multitasking
The `cotask` library provides a priority-based cooperative scheduler using Python
generators (`yield`). Each task suspends itself after one control update, giving
other tasks a chance to run. This avoids the complexity of preemptive RTOS while
still meeting timing requirements.

### Model-Based State Observer
Because the competition track requires dead-reckoning between line detections, we
designed a discrete-time Luenberger observer (4-state: center distance, heading,
left/right wheel angular velocity). The system matrices `A_D`, `B_D`, `C_D` were
derived in MATLAB and tuned for `T_s = 20 ms`. Inputs are motor voltages and encoder
distances; corrections use BNO055 heading and yaw rate.

### PI Control with Anti-Windup
All three PI loops (left motor, right motor, line follower) use the same
`PIController` class from `controller.py`, which implements conditional integration
anti-windup: the integrator state is only updated when the candidate output would
remain within saturation limits, or when the error is driving the output back toward
the feasible region.

### Feed-Forward on the Line-Follow Loop
During known-curvature segments (e.g., the 180° turn), a feed-forward term is added
to the line-follow output to pre-bias the differential velocity, reducing transient
error when the curve begins.
