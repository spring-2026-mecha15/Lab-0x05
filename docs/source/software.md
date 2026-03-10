# Software Architecture

## Repository Layout

```
Final Project/
├── main.py               # Hardware init, task instantiation, scheduler loop
├── constants.py          # Pin definitions, physical constants, system matrices
├── controller.py         # Generic PIController class
├── cotask.py             # Cooperative multitasking scheduler
├── task_share.py         # Inter-task Share and Queue primitives
├── task_motor.py         # Closed-loop motor speed task
├── task_line_follow.py   # Line-centering PI task
├── task_reflectance.py   # IR sensor array task
├── task_imu.py           # BNO055 IMU task
├── task_observer.py      # Luenberger state observer task
├── task_competition.py   # High-level competition sequencer task
├── task_user.py          # Serial user interface task
├── gains.json            # Persisted PI gains (written by user task)
├── imu.json              # Persisted IMU calibration
├── ir_calibration.json   # Persisted IR sensor calibration
└── desktop/
    ├── host.py           # PC-side serial data collector
    ├── plot.py           # Data plotter
    └── UI_Functions.py   # Desktop UI helpers
```

---

## Cooperative Scheduler (`cotask`)

All tasks are implemented as Python generator functions (using `yield`).
The `cotask.Task` wrapper tracks period and priority; `task_list.pri_sched()`
runs the highest-priority task that is ready on each scheduler tick.

```python
# Excerpt from main.py — task registration
task_list.append(Task(leftMotorTask.run,  priority=3, period=20))
task_list.append(Task(rightMotorTask.run, priority=4, period=20))
task_list.append(Task(observerTask.run,   priority=6, period=20))
```

---

## PI Controller (`controller.py`)

The `PIController` class is a reusable closed-loop controller used by both motor
tasks and the line-follow task.

**Anti-windup strategy:** conditional integration. The integrator is only
advanced when the candidate output stays within the saturation window, or when
the error is moving the saturated output back toward the feasible region. This
prevents integrator wind-up during motor stalls or tight turns.

**Transfer function (continuous-time equivalent):**

$$C(s) = K_p + \frac{K_i}{s}$$

**Control effort (discrete):**

$$u[k] = K_p \cdot e[k] + K_i \cdot \sum_{j=0}^{k} e[j] \cdot \Delta t + K_{ff} \cdot r_{ff}$$

---

## Motor Control (`task_motor.py`)

Each motor task runs at 20 ms (50 Hz). It:

1. Reads the encoder count delta and converts to wheel velocity (mm/s)
2. Runs the PI controller with the current setpoint
3. Writes the commanded voltage to the motor driver
4. Publishes voltage, wheel distance, and angular velocity shares for the observer

**Encoder gain:**

$$K_{enc} = \frac{2\pi r}{N_{counts}} \cdot 1000 \quad \left[\frac{\text{mm/s}}{\text{count/s}}\right]$$

where $r = 35\,\text{mm}$ and $N_{counts} = 1437.12$ counts/revolution.

---

## Line Following (`task_line_follow.py`)

The line-follow task reads the IR centroid share (computed by `task_reflectance`)
and drives a PI controller whose output is a differential velocity command:

```
left_setpoint  = nominal_speed + PI_output
right_setpoint = nominal_speed - PI_output
```

A feed-forward term is added during curved segments:

$$u_{ff} = K_{ff} \cdot \omega_{ref}$$

where $\omega_{ref}$ is the expected yaw rate for the known-radius turn.

---

## Reflectance Sensor (`task_reflectance.py`)

The 7-element QTRX array is read in three modes (controlled via `reflectanceMode` share):

| Mode | Meaning |
|------|---------|
| 0 | Idle |
| 1 | Dark calibration |
| 2 | Light calibration |
| 3 | Running — compute centroid |

The centroid is a weighted average of sensor indices, normalized so that
0 = line centered, negative = line to the left, positive = line to the right.

---

## IMU Task (`task_imu.py`)

The BNO055 IMU is read over I2C at 50 ms. It publishes absolute heading (radians)
and yaw rate (rad/s) to shared variables consumed by the observer.

---

## State Observer (`task_observer.py`)

A 4-state discrete-time Luenberger observer runs at 20 ms (50 Hz):

**State vector:**

$$\hat{x} = \begin{bmatrix} s_{center} \\ \psi \\ \dot{\theta}_L \\ \dot{\theta}_R \end{bmatrix}$$

where $s_{center}$ is distance traveled along the course centerline (mm),
$\psi$ is heading (rad), and $\dot{\theta}_{L,R}$ are wheel angular velocities (rad/s).

**Augmented input vector:**

$$u_{aug} = \begin{bmatrix} V_L \\ V_R \\ s_L \\ s_R \\ \psi \\ \dot{\psi} \end{bmatrix}$$

**Update equation:**

$$\hat{x}[k+1] = A_D \hat{x}[k] + B_D u_{aug}[k]$$
$$\hat{y}[k] = C_D \hat{x}[k]$$

The system matrices ($A_D$, $B_D$, $C_D$) were computed in MATLAB via pole placement
and are stored in `constants.py`.

---

## Competition Sequencer (`task_competition.py`)

The competition task is a finite-state machine (FSM) that coordinates the high-level
race strategy. It controls line-follow speed, feed-forward gain, and motor setpoints
based on the observer's center-distance estimate.

### State Transition Diagram

<!-- TODO: insert FSM diagram image -->
<!-- ![Competition FSM](../_static/competition_fsm.png) -->

| State | Name | Condition to advance |
|-------|------|---------------------|
| S0 | Idle | `competitionGo` flag set |
| S1 | Leg 1 — straight run with accel/decel profile | `observerCenterDistance ≥ LEG_1_DIST` (1375 mm) |
| S2 | Leg 2 — 180° curve with feed-forward | Line lost (end of curve) |
| S3 | Leg 3 — dead-reckoning through intermittent line | 314 mm traveled (≈ π × 100 mm arc) |
| S4 | Done — clear go flag | — |

**Velocity profile (Leg 1):**

$$v(t) = \begin{cases}
v_{start} + a \cdot t & s < s_{accel} \\
v_{max} & s_{accel} \le s < s_{decel} \\
\max(v_{start},\; v - a \cdot t) & s \ge s_{decel}
\end{cases}$$

with $a = 500\,\text{mm/s}^2$, $v_{max} = 500\,\text{mm/s}$, $v_{start} = 100\,\text{mm/s}$.

---

## User Interface (`task_user.py`)

The user task provides a serial menu (115200 baud) for:

- Tuning PI gains and saving them to `gains.json`
- Calibrating the IR sensor array (dark + light pass)
- Calibrating and loading IMU offsets
- Streaming real-time data (motor, centroid, observer) as CSV
- Starting/stopping the competition run

Data exported over serial is delimited by `>>>>CSV START<<<<` and `>>>>CSV END<<<<`
markers, which the desktop `host.py` script parses automatically.
