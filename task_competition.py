from micropython import const
from task_share import Share
from utime import ticks_ms, ticks_diff
import gc

S0 = const(0)     # Idle task. Wait for go command
S1 = const(1)     # Line follow with aggressive accel/decel
S2 = const(2)     
S3 = const(3)     # Line follow with feed-forward until line not detected
S4 = const(4)     # Continue velocity until Romi center at edge of parking garage (est.)
S5 = const(5)     # Blind radius CW for X distance until facing wall (est.)
S6 = const(6)     # Move forward until ultrasonic distance within threshold
S7 = const(7)     # Reverse (if needed) then rotate CCW until line found (and centroid ~= 0?)
S8 = const(8)     # Line follow until cross reached
S9 = const(9)     # Continue moving forward unti center aligned with cross
S10 = const(10)   # Turn CCW until line found
S11 = const(11)   # Move forward for X distance to move cup then reverse
S12 = const(12)   # Turn CW until line found
S13 = const(13)   # (might be unnecessary) Move forward until right corner hit
S14 = const(14)   # Line follow slalom. Retune Ki Kp for best consistency
S15 = const(15)   # Feed-forward line follow for X distance (R150)
S16 = const(16)   # Standard line follow for 50mm
S17 = const(17)   # Feed-forward line follow for X distance (R200)
S18 = const(18)   # Standard line follow for X distance
S19 = const(19)   # Perform preprogrammed cup push routine (CCW segment)
S20 = const(20)   # Perform preprogrammed cup push routine (CW segment)
S21 = const(21)   # Line follow for x distance
S22 = const(22)   # Blind radius for X distance
S23 = const(23)   # Enable line detection (once sensor is definitley only over right line)
                  # until line no longer found
S24 = const(24)   # (might be unnecessary) Move forward until dot found
S25 = const(25)   # Move forward to compensate for center-IR array offset

ACCELERATION = 500  # Acceleration in mm/s^2

LEG_1_DIST = 1375
LEG_1_ACCEL_DIST = 550
LEG_1_DECEL_DIST = 1000
LEG_1_START_VEL = 100
LEG_1_MAX_VEL = 500
LEG_1_END_VEL = 100

GARAGE_INSIDE_SPEED = 20.56
GARAGE_OUTSIDE_SPEED = 70.34

ROMI_LINE_OFFSET = 75 # Distance from Romi's center to line array

LEG_2_KFF = 0.6 # Feed forward gain for 200mm radius during leg 2

ROTATE_SPEED       = 75     # Wheel speed for in-place rotation (mm/s)
LINE_FOLLOW_SPEED  = 100    # Nominal line-follow speed for interior segments (mm/s)

S7_REVERSE_DIST    = 50     # Distance to reverse in S7 before rotating (mm)
S8_CROSS_DIST      = 100    # TBD: LF distance before cross detected in S8 (mm)

S11_CUP_PUSH_DIST  = 450    # Distance to drive forward to push cup (mm)
S11_REVERSE_DIST   = 450    # Distance to reverse after cup push (mm)

S13_FORWARD_DIST   = 1600   # TBD: short LF distance to clear right corner in S13 (mm)
S14_DIST           = 800    # TBD: slalom LF distance before transitioning to S15 (mm)

S15_DIST           = 157    # TBD: R150 feed-forward arc distance (mm)
S15_KFF            = 0.4    # TBD: Kff for R150 arc

S16_DIST           = 50     # Standard LF distance in S16 (mm)

S17_DIST           = 25    # TBD: R200 feed-forward arc distance (mm)
S17_KFF            = 0.3    # TBD: Kff for R200 arc

S18_DIST           = 50    # TBD: straight LF distance before cup push loop (mm)

S19_DIST           = 100    # TBD: CCW cup push loop segment distance (mm)
S20_DIST           = 400    # TBD: CW  cup push loop segment distance (mm)

S21_DIST           = 400    # TBD: LF distance from cup loop toward blind corner (mm)

S22_INSIDE_SPEED   = 20.56  # TBD: inside  wheel speed for blind radius to CP#4 (mm/s)
S22_OUTSIDE_SPEED  = 70.34  # TBD: outside wheel speed for blind radius to CP#4 (mm/s)
S22_DIST           = 300    # TBD: blind radius travel distance (mm)

S25_OFFSET_DIST    = 75     # Forward offset to center Romi over CP#5 (mm)

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

                    # self._lineFollowSetPoint.put(50)
                    # self._observerGoFlag.put(1)
                    # self._leftMotorGo.put(1)
                    # self._rightMotorGo.put(1)
                    # self._lineFollowGo.put(1)
                    # self._lineFollowKff.put(0)
                    # self._state = S1

                    # self._lineFollowGo.put(1)
                    # self._rightMotorGo.put(1)
                    # self._leftMotorGo.put(1)
                    # self._lineFollowSetPoint.put(LEG_1_END_VEL)
                    # self._lineFollowKff.put(LEG_2_KFF)
                    # self._state = S3 # PUT BACK WHEN DONE TESTING

                    # self._lineFollowGo.put(0)
                    # self._lineFollowKff.put(0)
                    # left_speed, right_speed = wheel_speeds(200, 75)
                    # self._leftMotorSetPoint.put(left_speed)
                    # self._rightMotorSetPoint.put(right_speed)
                    # self._rightMotorGo.put(1)
                    # self._leftMotorGo.put(1)
                    # self._state = S4

                    # left_speed = 75
                    # right_speed = 75
                    # self._leftMotorSetPoint.put(left_speed)
                    # self._rightMotorSetPoint.put(right_speed)
                    # self._lineFollowKff.put(0)
                    # self._state = S6

                    self._lineFollowGo.put(1)
                    self._rightMotorGo.put(1)
                    self._leftMotorGo.put(1)
                    # self._lineFollowSetPoint.put(LEG_1_END_VEL)
                    self._lineFollowKff.put(0)
                    self._state = S8 # PUT BACK WHEN DONE TESTING

            # Accelerate to top speed then decelerate before short radius
            elif self._state == S1:
                if not self._goFlag.get():
                    self._state = S0

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

            elif self._state == S2:
                self._state = S3

            # Follow short radius until line is no longer detected
            elif self._state == S3:
                # Continue to measure avg left/right velocities during radius

                if not self._lineFound.get():
                    self._lineFollowGo.put(0)
                    self._lineFollowKff.put(0)
                    left_speed, right_speed = wheel_speeds(200, 75)
                    self._leftMotorSetPoint.put(left_speed)
                    self._rightMotorSetPoint.put(right_speed)
                    self._centerStartDist = self._observerCenterDistance.get() # Reset distance reference
                    self._state = S4

            # Continue moving at same radius rate until center of romi is predicted
            # to be at the entrace edge of the parking garage
            elif self._state == S4:
                # Enforce motors run at avg radius speed
                segment_distance = self._observerCenterDistance.get() - self._centerStartDist
                # if (segment_distance) >= 314.1:
                if (segment_distance) >= 50:
                    # self._goFlag.put(0)
                    self._centerStartDist = self._observerCenterDistance.get() # Reset distance reference
                    # self._lineFollowGo.put(75)
                    # self._lineFollowKff.put(75)
                    left_speed, right_speed = wheel_speeds(125, 50)
                    self._leftMotorSetPoint.put(left_speed)
                    self._rightMotorSetPoint.put(right_speed)
                    self._state = S5

            elif self._state == S5:
                if (self._observerCenterDistance.get() - self._centerStartDist) >= 205:
                    self._centerStartDist = self._observerCenterDistance.get() # Reset distance reference
                    left_speed = 75
                    right_speed = 75
                    self._leftMotorSetPoint.put(left_speed)
                    self._rightMotorSetPoint.put(right_speed)
                    self._lineFollowKff.put(0)
                    self._state = S6

            elif self._state == S6:
                # if (self._observerCenterDistance.get() - self._centerStartDist) >= 375:
                u_dist = self._ultrasonicDistance.get()
                if u_dist < 20 and u_dist > 0:
                    self._leftMotorSetPoint.put(75/20 * u_dist)
                    self._rightMotorSetPoint.put(75/20 * u_dist)
                if u_dist < 5:
                    self._leftMotorSetPoint.put(0)
                    self._rightMotorSetPoint.put(0)
                    self._state = S7
            
            # --- S7: rotate CCW until line found ---
            elif self._state == S7:
                self._lineFollowGo.put(0)
                self._leftMotorSetPoint.put(-ROTATE_SPEED)
                self._rightMotorSetPoint.put(ROTATE_SPEED)
                self._centerStartDist = self._observerCenterDistance.get()
                self._state = S8

            elif self._state == S8:
                if self._lineFound.get() and self._centroid.get() <= -0.4:
                    self._centerStartDist = self._observerCenterDistance.get()
                    self._lineFollowSetPoint.put(75)
                    self._lineFollowGo.put(1)
                    self._state = S9
                    # self._state = S13

            elif self._state == S9:
                segment_distance = self._observerCenterDistance.get() - self._centerStartDist
                if segment_distance >= 250 and not self._lineFound.get():
                    self._lineFollowGo.put(0)
                    yield
                    # Search for line
                    left_speed, right_speed = wheel_speeds(30, 25)
                    self._leftMotorSetPoint.put(left_speed)
                    self._rightMotorSetPoint.put(right_speed)
                    self._state = S10

            elif self._state == S10:
                if self._lineFound.get() and (self._centroid.get() >= 0.2):
                    self._centerStartDist = self._observerCenterDistance.get()
                    self._lineFollowSetPoint.put(75)
                    self._lineFollowGo.put(1)
                    self._state = S13


            # --- S8: line follow until reaching the cross (~S8_CROSS_DIST) ---
            # elif self._state == S8:
                # segment_distance = self._observerCenterDistance.get() - self._centerStartDist
                # if segment_distance >= S8_CROSS_DIST:
                    # if not self._lineFound.get():
                        # self._state = S9

            # --- S9: creep forward ROMI_LINE_OFFSET to centre Romi over cross ---
            # elif self._state == S9:
            #     if (self._observerCenterDistance.get() - self._centerStartDist) >= ROMI_LINE_OFFSET:
            #         # Stop line follower; begin CCW rotation toward cup line
            #         self._lineFollowGo.put(0)
            #         self._leftMotorSetPoint.put(-ROTATE_SPEED)
            #         self._rightMotorSetPoint.put(ROTATE_SPEED)

            #         self._s9_phase += 1

            #         if self._s9_phase > 25:
            #             self._s9_phase = 0
            #             self._state = S10

            # # --- S10: rotate CCW in place until line found ---
            # elif self._state == S10:
            #     if self._lineFound.get():
            #         self._lineFollowSetPoint.put(LINE_FOLLOW_SPEED)
            #         self._lineFollowGo.put(1)
            #         self._centerStartDist = self._observerCenterDistance.get()
            #         self._s11_phase = 0
            #         self._state = S11

            # # --- S11: line-follow forward 450 mm to push cup, then reverse 450 mm ---
            # elif self._state == S11:
            #     if not self._lineFound.get():
            #         self._lineFollowGo.put(0)
            #         self._leftMotorSetPoint.put(ROTATE_SPEED)
            #         self._rightMotorSetPoint.put(ROTATE_SPEED)
            #     if self._s11_phase == 0:
            #         # Phase 0: line follow forward until cup push distance
            #         if (self._observerCenterDistance.get() - self._centerStartDist) >= S11_CUP_PUSH_DIST:
            #             self._lineFollowGo.put(0)
            #             self._leftMotorSetPoint.put(-ROTATE_SPEED)
            #             self._rightMotorSetPoint.put(-ROTATE_SPEED)
            #             self._centerStartDist = self._observerCenterDistance.get()
            #             self._s11_phase = 1

            #     elif self._s11_phase == 1:
            #         # Phase 1: reverse until S11_REVERSE_DIST covered
            #         if (self._centerStartDist - self._observerCenterDistance.get()) >= S11_REVERSE_DIST:
            #             # Begin CW rotation: left forward, right backward
            #             self._leftMotorSetPoint.put(ROTATE_SPEED)
            #             self._rightMotorSetPoint.put(-ROTATE_SPEED)
            #             self._s11_phase = 0
            #             self._state = S12

            # # --- S12: rotate CW in place until line found ---
            # elif self._state == S12:
            #     if self._lineFound.get():
            #         self._lineFollowSetPoint.put(LINE_FOLLOW_SPEED)
            #         self._lineFollowGo.put(1)
            #         self._centerStartDist = self._observerCenterDistance.get()
            #         self._state = S13

            # Slow down in anticipation for dashed line
            elif self._state == S13:
                if (self._observerCenterDistance.get() - self._centerStartDist) >= S13_FORWARD_DIST:
                    self._lineFollowSetPoint.put(30)
                    self._state = S14

            # Begin u-turn after dashed line
            elif self._state == S14:
                # if not self._lineFound.get():
                if (self._observerCenterDistance.get() - self._centerStartDist) >= S13_FORWARD_DIST:
                    self._lineFollowGo.put(0)
                    self._centerStartDist = self._observerCenterDistance.get()
                    left_speed, right_speed = wheel_speeds(-50, 50)
                    self._leftMotorSetPoint.put(left_speed)
                    self._rightMotorSetPoint.put(right_speed)
                    self._state = S15

            # After u-turn, go straight
            elif self._state == S15:
                if (self._observerCenterDistance.get() - self._centerStartDist) >= S15_DIST:
                    self._leftMotorSetPoint.put(75)
                    self._rightMotorSetPoint.put(75)
                    self._state = S16

            # Resume line following once line is picked back up
            elif self._state == S16:
                if self._lineFound.get():
                    self._centerStartDist = self._observerCenterDistance.get()
                    self._lineFollowGo.put(1)
                    self._lineFollowKff.put(0.3)
                    self._state = S17

            elif self._state == S17:
                # Must travel at least 25mm before testing if line has gone away
                if (self._observerCenterDistance.get() - self._centerStartDist) >= S17_DIST:
                    if not self._lineFound.get():
                        self._centerStartDist = self._observerCenterDistance.get()
                        self._lineFollowKff.put(0)
                        self._lineFollowGo.put(0)
                        left_speed, right_speed = wheel_speeds(200, 50)
                        self._leftMotorSetPoint.put(left_speed)
                        self._rightMotorSetPoint.put(right_speed)
                        self._state = S18

            elif self._state == S18:
                if (self._observerCenterDistance.get() - self._centerStartDist) >= S18_DIST:
                    self._centerStartDist = self._observerCenterDistance.get()
                    self._leftMotorSetPoint.put(50)
                    self._rightMotorSetPoint.put(50)
                    self._state = S19

            elif self._state == S19:
                if (self._observerCenterDistance.get() - self._centerStartDist) >= S19_DIST:
                    self._goFlag.put(0)
                    self._state = S0

            # --- S20: pre-programmed cup push loop, CW arc segment ---
            # elif self._state == S20:
            #     if (self._observerCenterDistance.get() - self._centerStartDist) >= S20_DIST:
            #         self._centerStartDist = self._observerCenterDistance.get()
            #         self._state = S21

            # # --- S21: line follow from cup loop toward blind corner ---
            # elif self._state == S21:
            #     if (self._observerCenterDistance.get() - self._centerStartDist) >= S21_DIST:
            #         self._lineFollowGo.put(0)
            #         self._leftMotorSetPoint.put(S22_OUTSIDE_SPEED)
            #         self._rightMotorSetPoint.put(S22_INSIDE_SPEED)
            #         self._centerStartDist = self._observerCenterDistance.get()
            #         self._state = S22

            # # --- S22: blind radius toward CP#4 return track ---
            # elif self._state == S22:
            #     self._leftMotorSetPoint.put(S22_OUTSIDE_SPEED)
            #     self._rightMotorSetPoint.put(S22_INSIDE_SPEED)
            #     if (self._observerCenterDistance.get() - self._centerStartDist) >= S22_DIST:
            #         self._lineFollowSetPoint.put(LINE_FOLLOW_SPEED)
            #         self._lineFollowGo.put(1)
            #         self._centerStartDist = self._observerCenterDistance.get()
            #         self._state = S23

            # # --- S23: line follow (right-side return track) until line lost ---
            # elif self._state == S23:
            #     if not self._lineFound.get():
            #         self._lineFollowGo.put(0)
            #         self._leftMotorSetPoint.put(LINE_FOLLOW_SPEED)
            #         self._rightMotorSetPoint.put(LINE_FOLLOW_SPEED)
            #         self._state = S24

            # # --- S24: drive forward until dot (line) re-detected ---
            # elif self._state == S24:
            #     if self._lineFound.get():
            #         self._centerStartDist = self._observerCenterDistance.get()
            #         self._leftMotorSetPoint.put(0)
            #         self._rightMotorSetPoint.put(0)
            #         self._state = S25

            # # --- S25: creep forward S25_OFFSET_DIST to centre Romi over CP#5 ---
            # elif self._state == S25:
            #     self._leftMotorSetPoint.put(LINE_FOLLOW_SPEED)
            #     self._rightMotorSetPoint.put(LINE_FOLLOW_SPEED)
            #     if (self._observerCenterDistance.get() - self._centerStartDist) >= S25_OFFSET_DIST:
            #         self._leftMotorSetPoint.put(0)
            #         self._rightMotorSetPoint.put(0)
            #         self._goFlag.put(0)
            #         self._state = S0

            yield

gc.collect()