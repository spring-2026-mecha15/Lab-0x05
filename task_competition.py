from micropython import const
from task_share import Share
from time import ticks_ms, ticks_diff

S0 = const(0)     # Idle task. Wait for go command
S1 = const(1)     # Run from point 0-1 (line follow with diverge)
S2 = const(2)     # Run from point 1-2 (line follow with 180 curve)
S3 = const(3)     # Run from point 2-3 (line follow with intermittent line)
S4 = const(4)     # Run from point 3-4 (line follow on "square wave")
S5 = const(5)     # Run from point 4-5 (parking garage)

S6 = const(6)     # Line follow from near wall to cross. Move center to cross
S7 = const(7)     # Turn left until line found and centroid ~0
S8 = const(8)
S9 = const(9)

ACCELERATION = 500  # Acceleration in mm/s^2

LEG_1_DIST = 1375
LEG_1_ACCEL_DIST = 550
LEG_1_DECEL_DIST = 1000
LEG_1_START_VEL = 100
LEG_1_MAX_VEL = 500
LEG_1_END_VEL = 100

ROMI_LINE_OFFSET = 75 # Distance from Romi's center to line array

LEG_2_KFF = 0.6 # Feed forward gain for 200mm radius during leg 2

class task_competition:
    def __init__(self,
            competitionGo:           Share,
            lineFollowGo:            Share,
            lineFollowSetPoint:      Share,
            lineFound:               Share,
            lineFollowKff:           Share,
            observerGoFlag:          Share,
            reflectanceMode:         Share,
            observerCenterDistance:  Share,
            observerHeading:         Share,
            leftMotorGo:             Share,
            leftMotorSetPoint:       Share,
            rightMotorGo:            Share,
            rightMotorSetPoint:      Share
        ):
        
        self._goFlag = competitionGo
        self._lineFollowGo = lineFollowGo
        self._lineFollowSetPoint = lineFollowSetPoint
        self._lineFound = lineFound
        self._lineFollowKff = lineFollowKff
        self._observerGoFlag = observerGoFlag
        self._reflectanceMode = reflectanceMode
        self._observerCenterDistance = observerCenterDistance
        self._observerHeading = observerHeading
        self._leftMotorGo = leftMotorGo
        self._leftMotorSetPoint = leftMotorSetPoint
        self._rightMotorGo = rightMotorGo
        self._rightMotorSetPoint = rightMotorSetPoint

        self._state = S0

        self._lastSegmentCenterDist = 0 # Mark the distance from center after the last segment
        self._startMs = ticks_ms()
        self._timer_ms = ticks_ms()

        self._leftMotorSpeedAvg = 0.0
        self._rightMotorSpeedAvg = 0.0

        self._velocity = 0

    def run(self):
        while True:
            if (self._state != 0) and (self._goFlag.get() == 0):
                self._leftMotorGo.put(0)
                self._rightMotorGo.put(0)
                self._observerGoFlag.put(0)
                self._lineFollowGo.put(0)
                self._reflectanceMode.put(0)
                self._state = 0

            if self._state == S0:
                if self._goFlag.get():
                    # Set initial configuration
                    self._timer_ms = ticks_ms()
                    self._centerStartDist = self._observerCenterDistance.get()
                    self._reflectanceMode.put(3)
                    self._velocity = LEG_1_START_VEL

                    self._lineFollowSetPoint.put(50)
                    # self._leftMotorSetPoint.put(50)
                    # self._rightMotorSetPoint.put(50)
                    # self._lineFollowSetPoint.put(self._velocity)
                    # self._leftMotorSetPoint.put(LEG_1_START_VEL)
                    # self._rightMotorSetPoint.put(LEG_1_START_VEL)

                    self._observerGoFlag.put(1)
                    self._leftMotorGo.put(1)
                    self._rightMotorGo.put(1)
                    self._lineFollowGo.put(1)
                    self._lineFollowKff.put(0)

                    # self._state = S1
                    self._state = S5 # PUT BACK WHEN DONE TESTING

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
                    self._leftMotorSpeedAvg = self._leftMotorSetPoint.get()
                    self._rightMotorSpeedAvg = self._rightMotorSetPoint.get()
                    self._centerStartDist = self._observerCenterDistance.get()
                    self._state = S2

            elif self._state == S2:
                # Continue to measure avg left/right velocities during radius
                self._leftMotorSpeedAvg += self._leftMotorSetPoint.get()
                self._leftMotorSpeedAvg /= 2
                self._rightMotorSpeedAvg += self._rightMotorSetPoint.get()
                self._rightMotorSpeedAvg /= 2

                if not self._lineFound.get():
                    self._lineFollowGo.put(0)
                    self._lineFollowKff.put(0)
                    yield
                    self._leftMotorSetPoint.put(self._leftMotorSpeedAvg)
                    self._rightMotorSetPoint.put(self._rightMotorSpeedAvg)
                    self._state = S3

            elif self._state == S3:
                # Enforce motors run at avg radius speed
                self._leftMotorSetPoint.put(self._leftMotorSpeedAvg)
                self._rightMotorSetPoint.put(self._rightMotorSpeedAvg)

                segment_distance = self._observerCenterDistance.get() - self._centerStartDist
                if (segment_distance) >= 314.1:
                    self._goFlag.put(0)
                    self._state = S4

            elif self._state == S4:
                self._goFlag.put(0) # Ack that competition is done
                self._state = S0

            elif self._state == S5:
                self._lineFollowGo.put(1)
                self._lineFollowSetPoint.put(50)
                self._leftMotorSetPoint.put(50)
                self._rightMotorSetPoint.put(50)
                self._lineFollowKff.put(0)
                self._state = S6

            elif self._state == S6:
                # if not self._lineFound.get():
                #     self._centerStartDist = self._observerCenterDistance.get()
                #     # self._state = S7
                #     self._state = S8
                self._centerStartDist = self._observerCenterDistance.get()
                self._state = S7
            
            elif self._state == S7:
                if (self._observerCenterDistance.get() - self._centerStartDist) >= ROMI_LINE_OFFSET:
                    self._state = S8
            
            elif self._state == S8:
                self._goFlag.put(0)
                self._state = S0
                pass
            

            yield