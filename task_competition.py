from micropython import const
from task_share import Share
from utime import ticks_ms, ticks_diff
import gc

S0 = const(0)
S1 = const(1)
S2 = const(2)
S3 = const(3)
S4 = const(4)
S5 = const(5)
S6 = const(6)
S7 = const(7)
S8 = const(8)
S9 = const(9)
S10 = const(10)
S11 = const(11)
S12 = const(12)
S13 = const(13)
S14 = const(14)

ACCELERATION = 500  # Acceleration in mm/s^2

LEG_1_DIST = 1375
LEG_1_ACCEL_DIST = 550
LEG_1_DECEL_DIST = 1000
LEG_1_START_VEL = 100
LEG_1_MAX_VEL = 500
LEG_1_END_VEL = 100

LEG_2_KFF = 0.6 # Feed forward gain for 200mm radius during leg 2

ROTATE_SPEED       = 75     # Wheel speed for in-place rotation (mm/s)
LINE_FOLLOW_SPEED  = 100    # Nominal line-follow speed for interior segments (mm/s)

S9_DIST           = 1800    # TBD: slalom LF distance before transitioning to S15 (mm)
S10_DIST           = 167    # TBD: R150 feed-forward arc distance (mm)
S12_DIST           = 25    # TBD: R200 feed-forward arc distance (mm)
S13_DIST           = 50    # TBD: straight LF distance before cup push loop (mm)
S14_DIST           = 100    # TBD: CCW cup push loop segment distance (mm)

def wheel_speeds(radius_mm, center_speed_mms, half_track=70.5):
    """Return (v_left, v_right) in mm/s. Positive radius = right turn."""
    return (center_speed_mms * (radius_mm + half_track) / radius_mm,
            center_speed_mms * (radius_mm - half_track) / radius_mm)

class task_competition:
    def __init__(self,
            competitionGo:           Share,
            lineFollowGo:            Share,
            lineFollowSetPoint:      Share,
            lineFound:               Share,
            lineFollowKff:           Share,
            lineCentroid:            Share,
            observerGoFlag:          Share,
            reflectanceMode:         Share,
            observerCenterDistance:  Share,
            observerHeading:         Share,
            leftMotorGo:             Share,
            leftMotorSetPoint:       Share,
            rightMotorGo:            Share,
            rightMotorSetPoint:      Share,
            ultrasonicDistance:      Share,
        ):
        
        self._goFlag = competitionGo
        self._lineFollowGo = lineFollowGo
        self._lineFollowSetPoint = lineFollowSetPoint
        self._lineFound = lineFound
        self._lineFollowKff = lineFollowKff
        self._centroid = lineCentroid
        self._observerGoFlag = observerGoFlag
        self._reflectanceMode = reflectanceMode
        self._observerCenterDistance = observerCenterDistance
        self._observerHeading = observerHeading
        self._leftMotorGo = leftMotorGo
        self._leftMotorSetPoint = leftMotorSetPoint
        self._rightMotorGo = rightMotorGo
        self._rightMotorSetPoint = rightMotorSetPoint
        self._ultrasonicDistance = ultrasonicDistance

        self._state = S0

        self._lastSegmentCenterDist = 0 # Mark the distance from center after the last segment
        self._startMs = ticks_ms()
        self._timer_ms = ticks_ms()

        self._velocity = 0

        self._s7_phase  = 0   # Sub-state for S7  (0=init, 1=reversing, 2=rotating)
        self._s11_phase = 0   # Sub-state for S11 (0=forward cup push, 1=reversing)
        self._s9_phase = 0

    def run(self):
        """
        Cooperative task for scheduler
        """
        left_speed = 0
        right_speed = 0

        while True:
            if (self._state != 0) and (not self._goFlag.get()):
                self._leftMotorGo.put(0)
                self._rightMotorGo.put(0)
                self._observerGoFlag.put(0)
                self._lineFollowGo.put(0)
                self._reflectanceMode.put(0)
                self._centerStartDist = self._observerCenterDistance.get()
                self._state = S0

            # Wait for go flag
            if self._state == S0:
                if self._goFlag.get():
                    # Set initial configuration
                    self._timer_ms = ticks_ms()
                    self._centerStartDist = self._observerCenterDistance.get()
                    self._reflectanceMode.put(3)
                    self._velocity = LEG_1_START_VEL

                    self._lineFollowSetPoint.put(50)
                    self._observerGoFlag.put(1)
                    self._leftMotorGo.put(1)
                    self._rightMotorGo.put(1)
                    self._lineFollowGo.put(1)
                    self._lineFollowKff.put(0)
                    self._state = S1

            # Accelerate to top speed then decelerate before short radius
            elif self._state == S1:
                dt = ticks_diff(ticks_ms(), self._timer_ms) / 1000
                self._timer_ms = ticks_ms()

                # Accel
                if self._observerCenterDistance.get() < LEG_1_ACCEL_DIST:
                    if self._velocity < LEG_1_MAX_VEL:
                        dv = ACCELERATION * dt
                        self._velocity += dv

                # Decel
                elif self._observerCenterDistance.get() >= LEG_1_DECEL_DIST:
                    dv = -1 * ACCELERATION * dt
                    if self._velocity > LEG_1_END_VEL:
                        self._velocity += dv

                self._lineFollowSetPoint.put(self._velocity)

                segment_distance = self._observerCenterDistance.get() - self._centerStartDist
                if (segment_distance) >= LEG_1_DIST:
                    self._lineFollowSetPoint.put(LEG_1_END_VEL)
                    # self._centerStartDist = self._observerCenterDistance.get() # NOTE: this isn't working. I don't know why...
                    self._lineFollowKff.put(LEG_2_KFF)
                    self._centerStartDist = self._observerCenterDistance.get()
                    self._state = S2

            # Follow short radius until line is no longer detected
            elif self._state == S2:
                # Continue to measure avg left/right velocities during radius

                if not self._lineFound.get():
                    self._lineFollowGo.put(0)
                    self._lineFollowKff.put(0)
                    left_speed, right_speed = wheel_speeds(200, 100)
                    self._leftMotorSetPoint.put(left_speed)
                    self._rightMotorSetPoint.put(right_speed)
                    self._centerStartDist = self._observerCenterDistance.get() # Reset distance reference
                    self._state = S3

            # Continue moving at same radius rate until center of romi is predicted
            # to be at the entrace edge of the parking garage
            elif self._state == S3:
                # Enforce motors run at avg radius speed
                segment_distance = self._observerCenterDistance.get() - self._centerStartDist
                # if (segment_distance) >= 314.1:
                if (segment_distance) >= 50:
                    # self._goFlag.put(0)
                    self._centerStartDist = self._observerCenterDistance.get() # Reset distance reference
                    # self._lineFollowGo.put(75)
                    # self._lineFollowKff.put(75)
                    left_speed, right_speed = wheel_speeds(125, 75)
                    self._leftMotorSetPoint.put(left_speed)
                    self._rightMotorSetPoint.put(right_speed)
                    self._state = S4

            # Once inside garage, turn 90deg right
            elif self._state == S4:
                if (self._observerCenterDistance.get() - self._centerStartDist) >= 205:
                    self._centerStartDist = self._observerCenterDistance.get() # Reset distance reference
                    left_speed = 100
                    right_speed = 100
                    self._leftMotorSetPoint.put(left_speed)
                    self._rightMotorSetPoint.put(right_speed)
                    self._lineFollowKff.put(0)
                    self._state = S5

            # Approach wall, slowing down as ultrasonic distance gets smaller
            # Once ultrasonic distance <~5cm, start rotating CCW to find line
            elif self._state == S5:
                u_dist = self._ultrasonicDistance.get()
                if u_dist < 20 and u_dist > 0:
                    self._leftMotorSetPoint.put(2.5 * u_dist + 50)
                    self._rightMotorSetPoint.put(2.5 * u_dist + 50)
                if u_dist < 5:
                    self._lineFollowGo.put(0)
                    self._leftMotorSetPoint.put(-ROTATE_SPEED)
                    self._rightMotorSetPoint.put(ROTATE_SPEED)
                    self._centerStartDist = self._observerCenterDistance.get()
                    self._state = S6
            
            # Once line picked up (and near center) resume line following
            elif self._state == S6:
                if self._lineFound.get() and self._centroid.get() <= -0.4:
                    self._centerStartDist = self._observerCenterDistance.get()
                    self._lineFollowSetPoint.put(100)
                    self._lineFollowGo.put(1)
                    self._state = S7
                    # self._state = S13

            # After romi has (est.) passed cross, prepare to lose the line at the sharp right turn
            # Once line is lost, start a sharp right turn to recover it
            elif self._state == S7:
                segment_distance = self._observerCenterDistance.get() - self._centerStartDist
                if segment_distance >= 250 and not self._lineFound.get():
                    self._lineFollowGo.put(0)
                    yield
                    # Search for line
                    left_speed, right_speed = wheel_speeds(30, 25)
                    self._leftMotorSetPoint.put(left_speed)
                    self._rightMotorSetPoint.put(right_speed)
                    self._state = S8

            # Once line is recovered (and near center) resume line following
            elif self._state == S8:
                # Enforce rotation in case line following overrode the wheel speeds.
                left_speed, right_speed = wheel_speeds(30, 25)
                self._leftMotorSetPoint.put(left_speed)
                self._rightMotorSetPoint.put(right_speed)

                if self._lineFound.get() and (self._centroid.get() >= 0.2):
                    self._centerStartDist = self._observerCenterDistance.get()
                    self._lineFollowSetPoint.put(75)
                    self._lineFollowGo.put(1)
                    self._state = S9

            # Begin u-turn after long distance (significantly past CP#4)
            elif self._state == S9:
                # if not self._lineFound.get():
                if (self._observerCenterDistance.get() - self._centerStartDist) >= S9_DIST:
                    self._lineFollowGo.put(0)
                    self._centerStartDist = self._observerCenterDistance.get()
                    left_speed, right_speed = wheel_speeds(-50, 50)
                    self._leftMotorSetPoint.put(left_speed)
                    self._rightMotorSetPoint.put(right_speed)
                    self._state = S10

            # After u-turn (based on center distance traveled), move straight to recover line following
            elif self._state == S10:
                if (self._observerCenterDistance.get() - self._centerStartDist) >= S10_DIST:
                    self._leftMotorSetPoint.put(75)
                    self._rightMotorSetPoint.put(75)
                    self._state = S11

            # Resume line following once line is picked back up
            elif self._state == S11:
                if self._lineFound.get():
                    self._centerStartDist = self._observerCenterDistance.get()
                    self._lineFollowGo.put(1)
                    self._lineFollowKff.put(0.3)
                    self._state = S12

            # When line is lost, move forward as provided radius to get center of romi at end of line
            elif self._state == S12:
                # Must travel at least 25mm before testing if line has gone away
                if (self._observerCenterDistance.get() - self._centerStartDist) >= S12_DIST:
                    if not self._lineFound.get():
                        self._centerStartDist = self._observerCenterDistance.get()
                        self._lineFollowKff.put(0)
                        self._lineFollowGo.put(0)
                        left_speed, right_speed = wheel_speeds(200, 50)
                        self._leftMotorSetPoint.put(left_speed)
                        self._rightMotorSetPoint.put(right_speed)
                        self._state = S13

            # Once romi is facing CP#5, move forward for a short distance
            elif self._state == S13:
                if (self._observerCenterDistance.get() - self._centerStartDist) >= S13_DIST:
                    self._centerStartDist = self._observerCenterDistance.get()
                    self._leftMotorSetPoint.put(50)
                    self._rightMotorSetPoint.put(50)
                    self._state = S14

            # After distance traveled, end test.
            elif self._state == S14:
                if (self._observerCenterDistance.get() - self._centerStartDist) >= S14_DIST:
                    self._goFlag.put(0)
                    self._state = S0

            yield

gc.collect()