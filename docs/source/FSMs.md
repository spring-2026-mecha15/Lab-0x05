# Finite State Machines (FSMs)

This project uses Finite State Machines (FSMs) implemented as Python generators operating within a cooperative scheduler. Each task's `run()` method yields the current state, allowing the scheduler to manage execution context.

## Summary & Interaction

- **[task_user](#user-interface-task-task-user-py)**: Command hub; enables/disables other tasks
- **[task_motor](#motor-control-task-task-motor-py)**: Two instances (left/right); controlled by goFlag from task_user or task_competition
- **[task_observer](#observerstate-estimator-task-task-observer-py)**: Provides center distance used by task_competition for segment tracking
- **[task_reflectance](#reflectance-sensor-task-task-reflectance-py)**: Provides centroid & line_found used by task_line_follow and task_competition
- **[task_line_follow](#line-following-task-task-line_follow-py)**: Steers motors based on centroid; gains & setpoint set by task_user or task_competition
- **[task_imu](#imu-task-task-imu-py)**: Provides heading used for rotation tracking; calibration managed via task_user
- **[task_ultrasonic](#ultrasonic-distance-task-task-ultrasonic-py)**: Provides distance; monitored by task_competition during S5 (garage approach)
- **[task_competition](#competition-course-task-task-competition-py)**: Orchestrates entire race; controls setpoints, enables/disables subsystems per segment

---

## 1. User Interface Task (`task_user.py`)

**Purpose:** Accept user commands, configure system parameters, and trigger subsystem operations.

### State Diagram

```
S0_PROMPT ──→ S1_CMD ──→ [Decision Point]
                         ├─→ S2_HELP ──→ S0_PROMPT
                         ├─→ S3_GAINS ──→ S0_PROMPT
                         ├─→ S4_SETPOINT ──→ S0_PROMPT
                         ├─→ S5_COLLECT ──→ S0_PROMPT
                         ├─→ S6_DEBUG ──→ S0_PROMPT
                         ├─→ S7_CALIBRATION ──→ S0_PROMPT
                         ├─→ S8_LINEFOLLOW ──→ S0_PROMPT
                         ├─→ S9_IMU_MENU ──→ S0_PROMPT
                         └─→ S10_STATE_ESTIMATION ──→ S0_PROMPT
```

### States

| State                    | Code | Purpose           | Behavior                                        |
| ------------------------ | ---- | ----------------- | ----------------------------------------------- |
| **S0_PROMPT**            | 0    | Display menu      | Print help menu once, transition to S1_CMD      |
| **S1_CMD**               | 1    | Await input       | Poll serial for single character command        |
| **S2_HELP**              | 2    | Show help         | Display help text, wait for Enter               |
| **S3_GAINS**             | 3    | Configure PID     | Import and delegate to `ui_gains` module        |
| **S4_SETPOINT**          | 4    | Set velocity      | Import and delegate to `ui_setpoint` module     |
| **S5_COLLECT**           | 5    | Data logging      | Collect motor velocity data while running test  |
| **S6_DEBUG**             | 6    | Debug menu        | Import and delegate to `ui_debug` module        |
| **S7_CALIBRATION**       | 7    | Calibrate sensor  | Import and delegate to `ui_calibration` module  |
| **S8_LINEFOLLOW**        | 8    | Line follow setup | Import and delegate to `ui_linefollow` module   |
| **S9_IMU_MENU**          | 9    | IMU control       | Import and delegate to `ui_imu` module          |
| **S10_STATE_ESTIMATION** | 10   | State observer    | Enable observer and line follower for debugging |

### Command Mapping (S1_CMD)

| Command | Action                | Destination                   |
| ------- | --------------------- | ----------------------------- |
| `h`     | Show help menu        | S2_HELP                       |
| `k`     | Enter PID gains       | S3_GAINS                      |
| `s`     | Set setpoint          | S4_SETPOINT                   |
| `g`     | Start motor test      | S5_COLLECT                    |
| `c`     | Calibrate reflectance | S7_CALIBRATION                |
| `i`     | Debug menu            | S6_DEBUG                      |
| `l`     | Line follow config    | S8_LINEFOLLOW                 |
| `e`     | State estimation      | S10_STATE_ESTIMATION          |
| `b`     | IMU menu              | S9_IMU_MENU                   |
| `m`     | Toggle memory monitor | (toggle flag, stay in S1_CMD) |
| Enter   | Return to menu        | S0_PROMPT                     |

### Key Behaviors

- **S3–S9:** Import UI modules, delegate execution via `yield from`, then unload module to save memory
- **S5_COLLECT:** Enables motor test, waits for queue saturation, then prints CSV data
- **S10_STATE_ESTIMATION:** Runs observer + line follower continuously, prints debug values each cycle

---

## 2. Motor Control Task (`task_motor.py`)

**Purpose:** Implement closed-loop PI velocity control for each motor with encoder feedback.

### State Diagram

```
S0_INIT → S1_WAIT ↔ S2_RUN
```

### States

| State       | Code | Purpose        | Behavior                                                                            |
| ----------- | ---- | -------------- | ----------------------------------------------------------------------------------- |
| **S0_INIT** | 0    | Initialization | Placeholder (can be removed); immediately transitions to S1_WAIT                    |
| **S1_WAIT** | 1    | Idle           | Monitor `goFlag` share; transition to S2_RUN when goFlag becomes non-zero           |
| **S2_RUN**  | 2    | Active control | Execute PI controller loop, update encoder state, collect profiling data if enabled |

### State Transitions

**S1_WAIT → S2_RUN**

- **Trigger:** `goFlag.get()` becomes non-zero
- **Initialization:**
  - Capture start timestamp (`startTime`)
  - Reset PI controller
  - Zero encoder position
  - Load gain values (Kp, Ki) and setpoint from shares
  - Enable motor driver (set voltage)

**S2_RUN → S1_WAIT**

- **Trigger:** `goFlag.get()` becomes zero
- **Cleanup:**
  - Disable motor (zero voltage)
  - Reset encoder

### S2_RUN Cycle Actions

Each iteration:

1. **Update Parameters** — Reload setpoint from `setpoint` share (allows real-time tuning)
2. **Capture Time** — Get current timestamp
3. **Encoder Update** — Read encoder, calculate velocity
4. **PI Control** — Call controller; updates motor voltage via callback
5. **Status Update** — Publish encoder position to `wheelDistance` share
6. **Velocity Publishing** — Compute angular velocity (ω = v / WHEEL_RADIUS), publish to `motorOmega` share
7. **Profiling** (if goFlag==2):
   - Extract velocity from encoder
   - Enqueue velocity → `dataValues` queue
   - Enqueue time (ms since start) → `timeValues` queue

### Testing

**Motor Test Mode:**

- User triggers via command `g` (S5_COLLECT in task_user)
- Sets `rightMotorGo=2` (profiling mode), `leftMotorGo=1` (normal mode)
- Data queues capture velocity over time for plot/analysis

---

## 3. Observer/State Estimator Task (`task_observer.py`)

**Purpose:** Estimate robot center position from dual encoder measurements.

### Current Status: **Simplified (No Active FSM)**

The FSM structure exists (S0_IDLE, S1_RUN) but is **commented out**. Current behavior is deterministic:

### States (Defined but Unused)

| State       | Code | Purpose                |
| ----------- | ---- | ---------------------- |
| **S0_IDLE** | 0    | Idle state (unused)    |
| **S1_RUN**  | 1    | Running state (unused) |

### Actual Behavior

Each cycle (no state branching):

1. **Calculate center distance:** `centerDistance = 0.5 × (leftWheelDist + rightWheelDist)`
2. **Publish** — Set `observerCenterDistance` share
3. **Yield** — Return control to scheduler

### Future Enhancement

Full observer implementation would include:

- Kalman filtering or discrete integration
- Heading-based position estimates
- Gyro/IMU integration for rotation tracking

---

## 4. Reflectance Sensor Task (`task_reflectance.py`)

**Purpose:** Manage line sensor calibration and continuous centroid tracking.

### State Diagram

```
S0_IDLE ←→ S1_CALIB_DARK
      ↓
      ←→ S2_CALIB_LIGHT
      ↓
      ←→ S3_RUN ←─┐
                   └─ (loop while running)
```

### States

| State              | Code | Purpose         | Trigger               |
| ------------------ | ---- | --------------- | --------------------- |
| **S0_IDLE**        | 0    | Idle            | Default state         |
| **S1_CALIB_DARK**  | 1    | Calibrate dark  | `reflectanceMode = 1` |
| **S2_CALIB_LIGHT** | 2    | Calibrate light | `reflectanceMode = 2` |
| **S3_RUN**         | 3    | Read centroid   | `reflectanceMode = 3` |

### State Transitions

**S0_IDLE Behavior**

- Poll `reflectanceMode` share:
  - `reflectanceMode = 0` → stay in S0_IDLE
  - `reflectanceMode = 1` → S1_CALIB_DARK
  - `reflectanceMode = 2` → S2_CALIB_LIGHT
  - `reflectanceMode = 3` → S3_RUN (save start time)
  - Other → raise ValueError

**S1_CALIB_DARK & S2_CALIB_LIGHT**

- **Entry:** Call `sensor.calibrate("dark")` or `sensor.calibrate("light")` → returns generator
- **Loop:** Iterate through calibration generator until completion
- **Exit:** When calibration generator exits (StopIteration)
- **Post-Exit:** Set `reflectanceMode = 0`, return to S0_IDLE

**S3_RUN Loop**

- **Entry:** Save `runStartTime` for CSV logging
- **Each Cycle:**
  1. Read sensor: `raw, calibrated, centroid, line_found = sensor.get_values()`
  2. Publish centroid → `lineCentroid` share
  3. Publish detection → `lineFound` share (boolean)
  4. If queues not full:
     - Enqueue centroid → `centroidValues`
     - Enqueue elapsed time (ms) → `centroidTimeValues`
- **Exit Condition:** `reflectanceMode` becomes 0
- **Post-Exit:** Return to S0_IDLE

### Data Format

- **Centroid:** float −1.0 to +1.0 (−1 = far left, 0 = center, +1 = far right)
- **Line Found:** boolean (True if contrast > threshold)

---

## 5. Line Following Task (`task_line_follow.py`)

**Purpose:** PI controller that steers robot to keep line at sensor center.

### State Diagram

```
State 0 (Idle) ↔ State 1 (Running)
```

### States

| State       | Code | Purpose        | Behavior                                                    |
| ----------- | ---- | -------------- | ----------------------------------------------------------- |
| **Idle**    | 0    | Waiting        | Monitor `lineFollowGo`; transition to Running on activation |
| **Running** | 1    | Active control | Execute PI controller each cycle using centroid as input    |

### State Transitions

**Idle → Running**

- **Trigger:** `lineFollowGo.get()` becomes non-zero
- **Initialization:**
  - Load **Kp, Ki** from `lineFollowKp`, `lineFollowKi` shares
  - Load nominal **setpoint** (straight-line speed in mm/s)
  - Calculate **feed-forward angular velocity** (assumes 300mm radius circle):
    ```
    omega_ff = nominalSetPoint / 300.0
    ```
  - Apply feed-forward gain: `controller.set_feed_forward(omega_ff, Kff)`
    - Kff initially 0; set externally during course following
  - Reset PI error integrator

**Running → Idle**

- **Trigger:** `lineFollowGo.get()` becomes zero
- **Cleanup:** Disable controller

### Running Cycle Actions

1. **Update Setpoint** — Reload straight-line speed from share (allows real-time tuning)
2. **Run Controller** — Call `controller.run()`
   - Automatically reads `lineCentroid` from share
   - Computes output (steering correction in mm/s)
   - Calls callback with output value
3. **Apply Output** — Callback `_plant_cb(output)` sets motor speeds:

   ```
   left_speed = nominalSetPoint + output
   right_speed = nominalSetPoint - output
   ```

   - **Positive output** (line on right) → slow right, speed up left (turn left)
   - **Negative output** (line on left) → slow left, speed up right (turn right)

### Feed-Forward Tuning

- **Initial value:** 0 (no FF)
- **Leg 2 (tight curve):** 0.6 (aggressive turn assistance)
- **Later segments:** 0.3 (gentle correction)
- **Set externally** via `lineFollowKff` share during competition course execution

---

## 6. IMU Task (`task_imu.py`)

**Purpose:** Initialize, configure, calibrate, and continuously read BNO055 9-DOF IMU sensor.

### State Diagram

```
S0_BEGIN
  ↓
S1_CALIBRATE (optional)
  ↓
S2_IDLE ←─────────────────┐
  ├→ S3_RUN_NDOF ────────┤
  ├→ S4_SAVE_CALIB ──────┤
  ├→ S5_LOAD_CALIB ──────┤
  ├→ S6_TARE ────────────┤
  ├→ S7_READ_VALS ───────┤
  └→ S8_GET_CALIB_STATE ─┘
```

### States

| State                  | Code | Purpose         | Mode Value                                   |
| ---------------------- | ---- | --------------- | -------------------------------------------- |
| **S0_BEGIN**           | 0    | Startup         | Initialization only                          |
| **S1_CALIBRATE**       | 1    | Auto-calibrate  | Wait for sys/gyro/accel/mag all = 0x03       |
| **S2_IDLE**            | 2    | Command wait    | Monitor mode share; dispatch to other states |
| **S3_RUN_NDOF**        | 3    | Stream data     | Continuous heading & rate output (mode=0xFF) |
| **S4_SAVE_CALIB**      | 4    | Save offsets    | Write IMU calibration to file (mode=2)       |
| **S5_LOAD_CALIB**      | 5    | Load offsets    | Read and apply calibration (mode=1)          |
| **S6_TARE**            | 6    | Tare accel/gyro | Capture zero-point references (mode=3)       |
| **S7_READ_VALS**       | 7    | One-shot read   | Read heading & rate once (mode=4)            |
| **S8_GET_CALIB_STATE** | 8    | Calib status    | Query and pack calibration flags (mode=5)    |

### State Transitions

**S0_BEGIN**

- **Actions:**
  1. Call `imu.initialize(NDOF_OP_MODE)` — cooperative initialization
  2. Attempt to load saved calibration via `_load_calibration()`
- **Conditional Exit:**
  - If load succeeds → skip S1_CALIBRATE, go to S2_IDLE
  - If load fails → (currently still goes to S2_IDLE; S1_CALIBRATE optional)

**S1_CALIBRATE** (Entry → S2_IDLE when calibrated)

- Poll calibration status each cycle
- If all sensors (sys, gyro, accel, mag) reach 0x03 (fully calibrated):
  - Save calibration via `_save_calibration()`
  - Transition to S2_IDLE

**S2_IDLE (Hub State)**

- Poll `imuMode` share:
  - `imuMode = 0` → stay in S2_IDLE
  - `imuMode = 1` → S5_LOAD_CALIB
  - `imuMode = 2` → S4_SAVE_CALIB
  - `imuMode = 3` → S6_TARE
  - `imuMode = 4` → S7_READ_VALS
  - `imuMode = 5` → S8_GET_CALIB_STATE
  - `imuMode = 0xFF` → S3_RUN_NDOF
  - Other → set flag, stay in S2_IDLE

**All Command States (S3–S8) → S2_IDLE**

- **Exit:** Set `imuMode = 0` (acknowledge command), return to S2_IDLE

### State Actions

**S3_RUN_NDOF** — Continuous Heading/Rate Output

- **First Entry Only:** Tare heading (capture zero reference)
- **Each Cycle:**
  1. Read gyro: `gx, gy, gz = imu.gyro()`
  2. Read Euler angles: `h, r, p = imu.euler()`
  3. Update `heading` share with h
  4. Update `headingRate` share with gz (yaw rate)
  5. Yield and return to S2_IDLE (recheck mode each cycle)

**S4_SAVE_CALIB** — Export Calibration

- Call `_save_calibration()`:
  - Get calibration offsets from IMU (22 bytes)
  - Get tare values (accel_xyz, gyro_xyz)
  - Write JSON to `IMU_FILE`
- Return to S2_IDLE

**S5_LOAD_CALIB** — Import Calibration

- Call `_load_calibration()`:
  - Read JSON from `IMU_FILE`
  - Extract offsets (22 bytes), apply via `imu.set_calibration_offsets()`
  - Extract & apply tare values (accel_xyz, gyro_xyz)
- Return to S2_IDLE

**S6_TARE** — Capture Zero Reference

- Call `imu.tare_accel_gyro()` (default: 100 samples, 5 ms delay)
  - Averages acceleration and gyro readings while stationary
  - Stores as offset in accel_tare, gyro_tare
  - Also saves to file via `_save_calibration()`
- Return to S2_IDLE

**S7_READ_VALS** — One-Shot Reading

- Read gyro, Euler angles
- Update `heading`, `headingRate` shares
- Return to S2_IDLE

**S8_GET_CALIB_STATE** — Query Calibration Flags

- Get `sys, gyro, accel, mag = imu.calibration_status()`
- Pack into one byte:
  ```
  result = (sys << 6) | (gyro << 4) | (accel << 2) | mag
  ```
- Update `imuCalibration` share with result
- Return to S2_IDLE

### Calibration Status Values

- **0** – Not calibrated
- **1** – Barely calibrated
- **2** – Good calibration
- **3** – Fully calibrated

---

## 7. Ultrasonic Distance Task (`task_ultrasonic.py`)

**Purpose:** Continuous distance measurement from ultrasonic sensor.

### Structure: **No FSM**

This task is a simple loop with no state machine:

```
Loop:
  1. Call sensor.loop()
  2. Read distance: d = sensor.get_distance()
  3. If d is None:
       ultrasonicDistance.put(0)
     Else:
       ultrasonicDistance.put(d)
  4. Yield
```

### Behavior

- **Runs indefinitely** — yields each cycle, no state transitions
- **Updates share:** `ultrasonicDistance` with measured distance in mm
- **Invalid readings:** Publishes 0 on sensor error/timeout

---

## 8. Competition Course Task (`task_competition.py`)

**Purpose:** Navigate a multi-leg autonomous course with line following, turns, curves, and obstacles.

### Course Layout

| Leg | Segment | Description                             | Distance |
| --- | ------- | --------------------------------------- | -------- |
| 1   | S1      | Accelerate/decelerate straight          | 1375 mm  |
| 2   | S2–S3   | Curved segment (R200 → R125)            | ~50+ mm  |
| 2   | S4–S5   | Garage approach (slow on wall approach) | ~205 mm  |
| 2   | S6–S7   | Exit garage, resume line follow         | Variable |
| 3   | S8–S9   | Long segment with U-turn                | ~1800 mm |
| 3   | S10–S11 | U-turn and re-acquisition               | Variable |
| 4   | S12–S14 | Final segments to finish                | ~150 mm  |

### State Diagram

```
S0 → S1 → S2 → S3 → S4 → S5 → S6 → S7 → S8 → S9 → S10 → S11 → S12 → S13 → S14 → S0
```

### States & Actions

| State   | Code | Entry Trigger  | Exit Condition          | Next State |
| ------- | ---- | -------------- | ----------------------- | ---------- |
| **S0**  | 0    | Reset/abort    | `competitionGoFlag = 1` | S1         |
| **S1**  | 1    | Race start     | Distance ≥ 1375 mm      | S2         |
| **S2**  | 2    | After accel    | Line lost               | S3         |
| **S3**  | 3    | Curve started  | Distance ≥ 50 mm        | S4         |
| **S4**  | 4    | Garage entry   | Distance ≥ 205 mm       | S5         |
| **S5**  | 5    | In garage      | Ultrasonic < 5 mm       | S6         |
| **S6**  | 6    | At wall        | Line + centroid         | S7         |
| **S7**  | 7    | Exit garage    | Distance + no line      | S8         |
| **S8**  | 8    | Sharp turn     | Line + centroid         | S9         |
| **S9**  | 9    | Resume follow  | Distance ≥ 1800 mm      | S10        |
| **S10** | 10   | Start U-turn   | Arc dist ≥ 167 mm       | S11        |
| **S11** | 11   | Complete turn  | Line found              | S12        |
| **S12** | 12   | Re-acquire     | Distance + no line      | S13        |
| **S13** | 13   | Final approach | Distance ≥ 50 mm        | S14        |
| **S14** | 14   | Final segment  | Distance ≥ 100 mm       | S0         |

### S0 (Idle/Abort)

**Entry Condition:** Initial state or abort detected

**Exit Condition:** `competitionGoFlag` becomes 1

**Initialization (on exit):**

- Start timer: save `ticks_ms()` for time-based acceleration ramps
- Save center distance reference: `_centerStartDist = observerCenterDistance.get()`
- Enable all subsystems:
  - `reflectanceMode = 3` (continuous line reading)
  - `observerGoFlag = 1` (enable state estimator)
  - `leftMotorGo = rightMotorGo = 1` (motors active)
  - `lineFollowGo = 1` (line follower active)
- Initialize motion:
  - Line follower setpoint = 100 mm/s
  - Feed-forward gain = 0

### S1 (Accelerate/Decelerate Leg)

**Entry:** From S0

**Distance Tracking:**

```
segmentDistance = observerCenterDistance.get() - _centerStartDist
```

**Acceleration Phase** (0–550 mm):

- If velocity < 500 mm/s:
  - `velocity += ACCELERATION × dt` (ramp up)
  - ACCELERATION = 500 mm/s²

**Deceleration Phase** (1000–1375 mm):

- If velocity > 100 mm/s:
  - `velocity -= ACCELERATION × dt` (ramp down)

**Each Cycle:**

1. Calculate elapsed time since last cycle
2. Apply accel/decel based on distance
3. Update line follower setpoint: `lineFollowSetPoint.put(velocity)`

**Exit Condition:** `segmentDistance ≥ 1375 mm`

**On Exit:**

- Set final velocity = 100 mm/s
- Enable feed-forward: `lineFollowKff = 0.6` (Leg 2 tight curve)
- Reset distance reference

### S2 (Follow Short Radius Curve, R200)

**Entry:** From S1

**Behavior:**

- Line follower disabled; motors run at fixed R200 radius:
  ```
  wheel_speeds(200, 100) → left_speed, right_speed
  ```

**Exit Condition:** Line is lost (`lineFound = False`)

**On Exit:**

- Disable line follower
- Set motors for tighter R125 curve
- Reset distance reference

### S3 (Move at Radius Speed, R125)

**Entry:** From S2

**Behavior:** Maintain fixed R125 radius motor speeds

**Exit Condition:** `segmentDistance ≥ 50 mm`

**On Exit:**

- Set motors equal forward: `left = right = 100` mm/s
- Clear feed-forward
- Reset distance reference

### S4 (Garage Forward)

**Entry:** From S3

**Behavior:** Move robot straight forward into garage at 100 mm/s

**Exit Condition:** `segmentDistance ≥ 205 mm`

**On Exit:**

- Reset motors to equal moderate speed
- Transition to S5

### S5 (Approach Wall, Ultrasonic Feedback)

**Entry:** From S4

**Ultrasonic Control:**

- If distance 20–100 mm:

  ```
  setpoint = 2.5 × distance + 50
  ```

  - Slow approach as distance decreases

- If distance < 5 mm:
  - Start rotation to find exit line
  - Set motors: `left = -75`, `right = +75` (CCW rotation)
  - Disable line follower
  - Return to S0 (abort) or transition to S6

### S6 (Rotate to Find Line)

**Entry:** From S5, CCW rotation active

**Exit Condition:** Line detected with centroid ≤ −0.4 (line on left edge)

**On Exit:**

- Enable line follower: setpoint = 100 mm/s
- Clear feed-forward
- Reset distance reference

### S7 (Line Follow Past Intersection)

**Entry:** From S6

**Behavior:** Continue line following past parking garage exit

**Exit Condition:** `segmentDistance ≥ 250 mm` AND line lost (`lineFound = False`)

**On Exit:**

- Disable line follower
- Set motors for sharp right turn: `wheel_speeds(30, 25)` (R30 radius)
- Reset distance reference

### S8 (Sharp Right Turn)

**Entry:** From S7

**Behavior:** Maintain R30 sharp turn radius (override any line follower)

**Each Cycle:** Enforce motor speeds via `wheel_speeds(30, 25)`

**Exit Condition:** Line found with centroid ≥ 0.2 (line re-acquired on right)

**On Exit:**

- Enable line follower: setpoint = 75 mm/s
- Set feed-forward = 0.3 (gentle assistance)
- Reset distance reference

### S9 (Long Straight Segment, Prepare U-Turn)

**Entry:** From S8

**Behavior:** Continue line following on long segment

**Exit Condition:** `segmentDistance ≥ 1800 mm`

**On Exit:**

- Disable line follower
- Set motors for U-turn: `wheel_speeds(-50, 50)` (left negative, right positive)
  - Creates rotation in place
- Reset distance reference

### S10 (Perform U-Turn)

**Entry:** From S9

**Behavior:** Execute U-turn via arc motion

**Calculate Arc Distance:**

```
arc_angle = current_heading - start_heading
arc_distance = arc_angle × EFFECTIVE_RADIUS
```

**Exit Condition:** `segmentDistance ≥ 167 mm` (arc distance completed)

**On Exit:**

- Set motors forward: `left = right = 75` mm/s
- Reset distance reference

### S11 (Resume Line Following After U-Turn)

**Entry:** From S10

**Behavior:** Move forward slowly, waiting for line re-detection

**Exit Condition:** Line detected (`lineFound = True`)

**On Exit:**

- Enable line follower: setpoint = 75 mm/s, Kff = 0.3
- Reset distance reference

### S12 (Short Radius Curve After Line Recovery)

**Entry:** From S11

**Minimum Distance:** Must travel ≥ 25 mm before checking for line loss (avoid immediate re-loss oscillation)

**Behavior:** Line follow for 25 mm, then check for line loss

**Exit Condition:** `segmentDistance ≥ 25 mm` AND `lineFound = False`

**On Exit:**

- Disable line follower & feed-forward
- Set motors for R200 radius: `wheel_speeds(200, 50)`
- Reset distance reference

### S13 (Final Approach to Checkpoint)

**Entry:** From S12

**Behavior:** Maintain R200 radius curve toward finish

**Exit Condition:** `segmentDistance ≥ 50 mm`

**On Exit:**

- Set motors: `left = right = 50` mm/s (slow forward)
- Reset distance reference

### S14 (Final Segment to Finish)

**Entry:** From S13

**Behavior:** Final forward motion to cross finish line

**Exit Condition:** `segmentDistance ≥ 100 mm`

**On Exit:**

- Set `competitionGoFlag = 0` (stop race)
- Return to S0

### Distance Tracking & Reset Pattern

All segments use the same distance calculation:

```
segmentDistance = observerCenterDistance.get() - _centerStartDist
```

At each state transition, `_centerStartDist` is updated to the current center distance, effectively resetting the segment counter.

### Subsystem Activation

| Subsystem     | S0  | S1  | S2–S14     |
| ------------- | --- | --- | ---------- |
| Reflectance   | 0   | 3   | 3 (varies) |
| Observer      | 0   | 1   | 1          |
| Motors        | 0   | 1   | 1          |
| Line Follower | 0   | 1   | varies     |

---
