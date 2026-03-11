from micropython import const
from task_share import Share
from ulab import numpy as np

from constants import A_D, B_D, C_D, D_D

S0_IDLE = const(0)
S1_RUN  = const(1)

class task_observer:
    def __init__(
            self,
            goFlag:                 Share,
            distLeft:                Share,
            distRight:               Share,
            voltageLeft:             Share,
            voltageRight:            Share,
            heading:                 Share,
            headingRate:             Share,
            observerCenterDistance:  Share,
            observerHeading:         Share,
            observerHeadingRate:     Share,
            observerOmegaLeft:       Share,
            observerOmegaRight:      Share,
            observerDistanceLeft:    Share,
            observerDistanceRight:   Share
        ):

        self._goFlag = goFlag

        self.s_L = distLeft
        self.s_R = distRight

        self.u_L = voltageLeft
        self.u_R = voltageRight

        self.psi = heading
        self.psi_dot = headingRate

        self.A_D = np.array(A_D)
        self.B_D = np.array(B_D)
        self.C_D = np.array(C_D)
        
        self._observerCenterDistance = observerCenterDistance
        self._observerHeading = observerHeading
        self._observerHeadingRate = observerHeadingRate
        self._observerOmegaLeft = observerOmegaLeft
        self._observerOmegaRight = observerOmegaRight
        self._observerDistanceLeft = observerDistanceLeft
        self._observerDistanceRight = observerDistanceRight


        self.x_hat = np.array([[0.0], [0.0], [0.0], [0.0]])
        self.y_hat = np.array([[0.0], [0.0], [0.0], [0.0]])
        self.u_aug = np.array([[0.0], [0.0], [0.0], [0.0], [0.0], [0.0]])

        self._state = S0_IDLE

    @property
    def s_L(self):
        return self.__dict__.get('s_L')

    @s_L.setter
    def s_L(self, value):
        self.__dict__['s_L'] = value

    @property
    def s_R(self):
        return self.__dict__.get('s_R')

    @s_R.setter
    def s_R(self, value):
        self.__dict__['s_R'] = value

    @property
    def u_L(self):
        return self.__dict__.get('u_L')

    @u_L.setter
    def u_L(self, value):
        self.__dict__['u_L'] = value

    @property
    def u_R(self):
        return self.__dict__.get('u_R')

    @u_R.setter
    def u_R(self, value):
        self.__dict__['u_R'] = value

    @property
    def psi(self):
        return self.__dict__.get('psi')

    @psi.setter
    def psi(self, value):
        self.__dict__['psi'] = value

    @property
    def psi_dot(self):
        return self.__dict__.get('psi_dot')

    @psi_dot.setter
    def psi_dot(self, value):
        self.__dict__['psi_dot'] = value

    @property
    def A_D(self):
        return self.__dict__.get('A_D')

    @A_D.setter
    def A_D(self, value):
        self.__dict__['A_D'] = value

    @property
    def B_D(self):
        return self.__dict__.get('B_D')

    @B_D.setter
    def B_D(self, value):
        self.__dict__['B_D'] = value

    @property
    def C_D(self):
        return self.__dict__.get('C_D')

    @C_D.setter
    def C_D(self, value):
        self.__dict__['C_D'] = value

    @property
    def x_hat(self):
        return self.__dict__.get('x_hat')

    @x_hat.setter
    def x_hat(self, value):
        self.__dict__['x_hat'] = value

    @property
    def y_hat(self):
        return self.__dict__.get('y_hat')

    @y_hat.setter
    def y_hat(self, value):
        self.__dict__['y_hat'] = value

    @property
    def u_aug(self):
        return self.__dict__.get('u_aug')

    @u_aug.setter
    def u_aug(self, value):
        self.__dict__['u_aug'] = value

    def update(self, u_L, u_R, s_L, s_R, psi, psi_dot):
        
        # Updates the state estimate using the latest motor commands and sensor readings.
        # Must be called at exactly Ts = 0.02s (50 Hz).
        
        # State Estimation Parameters:
        # -----------
        # u_L     : float : Commanded left motor voltage (Volts) 
        # u_R     : float : Commanded right motor voltage (Volts)
        # s_L     : float : Measured left wheel distance from encoders (mm)
        # s_R     : float : Measured right wheel distance from encoders (mm)
        # psi     : float : Measured heading/yaw from BNO055 IMU (radians)
        # psi_dot : float : Measured yaw rate from BNO055 IMU (radians/sec)
        
        # Returns:
        # --------
        # x_hat   : ulab.numpy.ndarray : The new 4x1 state estimate vector

        self.u_aug[0][0] = u_L
        self.u_aug[1][0] = u_R
        self.u_aug[2][0] = s_L
        self.u_aug[3][0] = s_R
        self.u_aug[4][0] = psi
        self.u_aug[5][0] = psi_dot
        
        self.x_hat = np.dot(self.A_D, self.x_hat) + np.dot(self.B_D, self.u_aug)
        self.y_hat = np.dot(self.C_D, self.x_hat)

        return
    
    def run(self):
        while True:
            if self._state == S0_IDLE:
                if self._goFlag.get(S0_IDLE):
                    self._state = S1_RUN

            elif self._state == S1_RUN:
                if not self._goFlag.get(S0_IDLE):
                    self._state = S0_IDLE


                else:
                    self.update(
                        self.u_L.get(),
                        self.u_R.get(),
                        self.s_L.get(),
                        self.s_R.get(),
                        self.psi.get(),
                        self.psi_dot.get(),
                    )
                    # Publish observer states as scalar shares.
                    self._observerCenterDistance.put(float(self.x_hat[0][0]))
                    self._observerDistanceLeft.put(float(self.y_hat[0][0]))
                    self._observerDistanceRight.put(float(self.y_hat[1][0]))
                    self._observerHeading.put(float(self.y_hat[2][0]))
                    self._observerHeadingRate.put(float(self.y_hat[3][0]))
                    self._observerOmegaLeft.put(float(self.x_hat[2][0]))
                    self._observerOmegaRight.put(float(self.x_hat[3][0]))
            yield 0
