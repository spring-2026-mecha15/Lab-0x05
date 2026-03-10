# Results & Demo

## Video Demonstration

<!-- TODO: replace YOUR_VIDEO_ID with the actual YouTube video ID -->
<!--
<iframe width="700" height="394"
  src="https://www.youtube.com/embed/YOUR_VIDEO_ID"
  title="Mecha15 Demo"
  frameborder="0"
  allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture"
  allowfullscreen>
</iframe>
-->

*Video coming soon — link will be embedded here.*

---

## Competition Performance

<!-- TODO: fill in actual run results -->

| Metric | Value |
|--------|-------|
| Best competition time | TBD |
| Max speed achieved | ~500 mm/s |
| Legs completed | TBD / 4 |

---

## Tuning Results

### Motor PI Gains

| Parameter | Value |
|-----------|-------|
| $K_p$ | 0.15 |
| $K_i$ | 4.0 |

### Line-Follow PI Gains

| Parameter | Value |
|-----------|-------|
| $K_p$ | 0.40 |
| $K_i$ | 0.30 |
| $K_{ff}$ | 0.60 (curve segments) |

---

## Step Response Plots

<!-- TODO: embed plots from desktop/data/ -->
<!-- ![Motor Step Response](../_static/step_response.png) -->
<!-- ![Centroid vs Time](../_static/centroid_plot.png) -->

The desktop `plot.py` script generates plots from CSV data streamed off the robot.
Raw data files are stored in `desktop/data/`.

---

## Observations & Lessons Learned

<!-- TODO: reflect on what worked, what didn't, and what you'd change -->

### What Worked Well
- The anti-windup PI controller responded cleanly to step velocity commands
- BNO055 heading was stable enough for short dead-reckoning segments
- Feed-forward on the 180° turn significantly reduced overshoot

### Challenges
- Memory constraints on MicroPython required careful import ordering and `gc.collect()` calls
- The observer center-distance estimate drifted during the intermittent-line segment
- Initial IMU calibration procedure was time-consuming

### Future Improvements
- Integrate a proper Kalman filter with measurement noise tuning
- Add a wall-detection sensor for the parking garage maneuver
- Implement closed-loop heading control for the dead-reckoning segments
