# Results & Reflection

## Time Trial Performance

The robot was able to perform very well in the time trial. It was able to complete the course consistantly in under 55.7 seconds, with only a 0.2 second deviation between the slowest and fastest run. The time breakdown for the fastest trial is listed below.

| Checkpoint | Time (seconds) | 
|---|---|
| 1 | 3.3 |
| 2 | 19.61 |
| 3 | 34.73 |
| 4 | 44.12 |
| Finish | 55.5 |

The total time for each trial is listed below.

| Trial | Time (seconds) | 
|---|---|
| 1 | 55.6 |
| 2 | 55.7 |
| 3 | 55.5 |


---

## Reflection

Overall, we are very happy with how this project came together. Roami's course performance was both fast and accurate, completing the course consistently in under 55.7 seconds with only a 0.2-second spread across all three runs.

**What we'd do differently:** The biggest lesson was committing to a design decision early and sticking with it. We invested significant time integrating the BNO055 IMU and developing a Luenberger state observer, only to replace both with a simple wheel-average estimator for the final time trial. In hindsight, we would have either committed to the IMU and state estimation approach from the start, or cut it early and redirected that effort elsewhere.

**Challenges:** Limited RAM on the STM32 forced us to develop a dynamic approach to the serial UI, loading and unloading submenus from RAM on demand rather than holding all menu state in memory simultaneously, including a dedicated submenu for BNO055 IMU calibration. The most time-consuming debugging experience was tracing an intermittently unresponsive UI. After multiple days of investigation, we found the bug had two independent, concurrent causes: RAM exhaustion under certain task, and an echo pin on the HC-SR04 ultrasonic sensor being connected to pin C17 which controls the user button. Neither was easily identifiable, and their interaction made the failure mode inconsistent and hard to reproduce.

**What we're most proud of:** One of the elements we are most proud of is the the line-following performance. The robot stayed centered through tight curves and long straights, with the PI and feed-forward controller working together cleanly without oscillation. We are also proud of the battery droop compensation system, which scales motor effort in real time based on measured pack voltage, ensuring consistent speed and behavior regardless of battery voltage.