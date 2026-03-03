from task_share import Share
from ulab import numpy as np

from constants import A_D, B_D, C_D, D_D

class task_observer:
    def __init__(
            self,
            distLeft:      Share,
            distRight:     Share,
            voltageLeft:    Share,
            voltageRight:   Share,
            heading:       Share,
            headingRate:   Share
        ):

        self.s_L = distLeft
        self.s_R = distRight

        self.u_L = voltageLeft
        self.u_R = voltageRight

        self.psi = heading
        self.psi_dot = headingRate

        self.A_D = np.array(A_D)
        self.B_D = np.array(B_D)
        self.C_D = np.array(C_D)
        
        self.x_hat = np.array([[0.0], [0.0], [0.0], [0.0]])
        self.y_hat = np.array([[0.0], [0.0], [0.0], [0.0]])

    def update(self, u_L, u_R, s_L, s_R, psi, psi_dot):
        
        """
        Updates the state estimate using the latest motor commands and sensor readings.
        Must be called at exactly Ts = 0.02s (50 Hz).
        
        State Estimation Parameters:
        -----------
        u_L     : float : Commanded left motor voltage (Volts) 
        u_R     : float : Commanded right motor voltage (Volts)
        s_L     : float : Measured left wheel distance from encoders (mm)
        s_R     : float : Measured right wheel distance from encoders (mm)
        psi     : float : Measured heading/yaw from BNO055 IMU (radians)
        psi_dot : float : Measured yaw rate from BNO055 IMU (radians/sec)
        
        Returns:
        --------
        x_hat   : ulab.numpy.ndarray : The new 4x1 state estimate vector
        """

        u_aug = np.array([
            [u_L], [u_R], [s_L], [s_R], [psi], [psi_dot]
        ])
        
        self.x_hat = np.dot(self.A_D, self.x_hat) + np.dot(self.B_D, u_aug)
        
        # Explicitly calculate y_hat for data logging/plotting
        self.y_hat = np.dot(self.C_D, self.x_hat)
        
        return self.x_hat, self.y_hat
    
    def run(self):
        while True:
            #need pull shares then do unit conversions before passing into the update function
            self.update(
                self.u_L.get(),
                self.u_R.get(),
                self.s_L.get(),
                self.s_R.get(),
                self.psi.get(),
                self.psi_dot.get(),
            )
            yield 0
