from task_share import Share
from controller import PIController
try:
    import ujson as json
except ImportError:
    import json
from constants import GAINS_FILE, DEFAULT_LF_KP, DEFAULT_LF_KI, DEFAULT_LF_KFF

class task_line_follow:
    def __init__(
            self,
            lineFollowGo:       Share,
            lineFollowSetPoint: Share,
            lineFollowKp:       Share,
            lineFollowKi:       Share,
            lineCentroid:       Share,
            lineFollowKff:      Share,
            rightMotorSetPoint: Share,
            leftMotorSetPoint:  Share
        ):
        
        self._state               = 0

        self._goFlag              = lineFollowGo

        self._setPoint            = lineFollowSetPoint

        self._Kp                  = lineFollowKp

        self._Ki                  = lineFollowKi

        self._lineCentroid        = lineCentroid

        self._Kff                 = lineFollowKff

        self._nominalSetPoint     = 0

        self._rightMotorSetPoint  = rightMotorSetPoint

        self._leftMotorSetPoint   = leftMotorSetPoint

        self._controller = PIController(        # Instantiate a PI controller to affect Romi spin
            self.plant_cb,                      # as the line centroid drifts from center.
            141,                                # Velocity at wheels = W*Omega, where W is wheel width (141mm)
            self._lineCentroid.get,
            1,
            (-100, 100)
        )

        self._load_gains()

        # Set Controller Parameters
        self._controller.Kp = lineFollowKp.get() # Proportional gain
        self._controller.Ki = lineFollowKi.get() # Integral gain

        self._controller.set_point = 0           # Setpoint is 0: corresponding to 
                                                 # keeping the line centered under romi

        print("Line follower task instantiated")

    def _load_gains(self) -> bool:
        try:
            with open(GAINS_FILE, "r") as gains_file:
                data = json.load(gains_file)
        except (OSError, ValueError):
            return False

        line_follower = data.get("line_follower", {})
        lf_kp = line_follower.get("kp")
        lf_ki = line_follower.get("ki")
        lf_setpoint = line_follower.get("set_point")
        lf_kff = line_follower.get("kff")

        if lf_kp is not None:
            self._Kp.put(float(lf_kp))
            print(f"Read LF Kp: {float(lf_kp)}")
        else:
            self._Kp.put(DEFAULT_LF_KP)
            print(f"LF Kp not found. Using default: {DEFAULT_LF_KP}")
        if lf_ki is not None:
            self._Ki.put(float(lf_ki))
            print(f"Read LF Ki: {float(lf_ki)}")
        else:
            self._Ki.put(DEFAULT_LF_KI)
            print(f"LF Ki not found. Using default: {DEFAULT_LF_KI}")
        if lf_setpoint is not None:
            self._nominalSetPoint = float(lf_setpoint)
            self._setPoint.put(float(lf_setpoint))
            print(f"Read LF setpoint: {float(lf_setpoint)}")

        if lf_kff is not None:
            self._Kff.put(lf_kff)
            print(f"Read LF Kff: {float(lf_kff)}")
        else:
            self._Ki.put(DEFAULT_LF_KFF)
            print(f"LF Kff not found. Using default: {DEFAULT_LF_KFF}")
        

        return True

    def run(self):
        # Track previous goflag state to determine when flag was just set
        oldGo = False

        while True:
            if self._goFlag.get():

                # Update line follow gains once each time go flag is set
                if not(oldGo):
                    self._controller.Kp = self._Kp.get()
                    self._controller.Ki = self._Ki.get()
                    self._nominalSetPoint = self._setPoint.get() # Setpoint in mm/s

                    # Configure feed-forward for this velocity
                    radius = 300 # radius of test circle in mm
                    omega = self._nominalSetPoint / radius
                    self._controller.set_feed_forward(omega, self._Kff.get())
                    

                    self._controller.reset()
                    
                    oldGo = True
                
                # Run line follower PI control
                # input will be read from the centroid share
                # output will be sent to `plant_cb`
                self._controller.run()

            else:
                oldGo = False

            yield self._state

    def plant_cb(self, value):
        # print(f'{self._count} - Plant rx value: {value:.3f}')
        self._leftMotorSetPoint.put(self._nominalSetPoint + value)
        self._rightMotorSetPoint.put(self._nominalSetPoint - value)
