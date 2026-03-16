"""
task_observer.py — Cooperative state-observer task for a 2-wheeled
differential-drive robot.

Implements a discrete-time Luenberger observer that estimates the full
robot state from sensor measurements each control cycle.  The task is
designed for MicroPython's cooperative scheduler: calling ``run()``
returns a generator that yields control back to the scheduler after
every update.

State vector x_hat (4 elements):
    [center_distance, heading, omega_left, omega_right]

Input vector u (6 elements):
    [voltage_left, voltage_right, dist_left, dist_right, heading, heading_rate]

Output vector y_hat (4 elements):
    [dist_left, dist_right, heading, heading_rate]
"""

from micropython import const
from task_share import Share
from array import array


# Discrete-time observer matrices computed offline (e.g., via MATLAB or a
# Python control-design script) for the robot's linearised model.
# Stored as flat row-major tuples so matrix-vector products can be performed
# with pure integer indexing — no NumPy/ulab required, no heap allocation.

# A_D: 4x4 discrete state-transition matrix (x_k+1 = A_D @ x_k + ...)
_A_D = (
     0.3799498,  0.0000000,  0.3486384,  0.3486384,
     0.0000000,  0.0000029,  0.0000000,  0.0000000,
    -0.1693591,  0.0000000,  0.1102907,  0.1102815,
    -0.1693591,  0.0000000,  0.1102815,  0.1102907,
)
# B_D: 4x6 discrete input matrix (... + B_D @ u_k)
_B_D = (
     0.8657503,  0.8657503,  0.3100251,  0.3100251,  0.0000000,  0.0000000,
     0.0000000,  0.0000000, -0.0071421,  0.0071421,  0.0001020,  0.0039210,
     1.1426413,  0.8415850,  0.0846795,  0.0846795,  0.0000000, -1.8274565,
     0.8415850,  1.1426413,  0.0846795,  0.0846795,  0.0000000,  1.8274565,
)
# C_D: 4x4 discrete output matrix (y_hat = C_D @ x_hat)
_C_D = (
     1.0000000, -70.0000000,  0.0,        0.0,
     1.0000000,  70.0000000,  0.0,        0.0,
     0.0,         1.0000000,  0.0,        0.0,
     0.0,         0.0,       -0.2500000,  0.2500000,
)

S0_IDLE = const(0)
S1_RUN  = const(1)


class task_observer:
    """
    Cooperative Luenberger observer task for a differential-drive robot.

    Each call to ``run()`` yields a generator suitable for use with a
    round-robin cooperative scheduler.  On every active cycle the observer
    propagates the state estimate forward one step and publishes the results
    to shared inter-task variables.

    All matrix operations use pre-allocated ``array.array`` buffers and flat
    tuple indexing to avoid heap allocation in the 50 Hz hot path.
    """

    def __init__(
            self,
            goFlag:                 Share,
            distLeft:               Share,
            distRight:              Share,
            voltageLeft:            Share,
            voltageRight:           Share,
            heading:                Share,
            headingRate:            Share,
            observerCenterDistance: Share,
            observerHeading:        Share,
            observerHeadingRate:    Share,
            observerOmegaLeft:      Share,
            observerOmegaRight:     Share,
            observerDistanceLeft:   Share,
            observerDistanceRight:  Share
        ):
        """
        Initialise the observer task.

        Parameters
        ----------
        goFlag : Share
            Boolean flag set by the supervisor task; the observer runs only
            while this flag is True.
        distLeft : Share
            Measured left-wheel travel distance (m).
        distRight : Share
            Measured right-wheel travel distance (m).
        voltageLeft : Share
            Applied left-motor voltage (V).
        voltageRight : Share
            Applied right-motor voltage (V).
        heading : Share
            IMU-measured heading angle psi (rad).
        headingRate : Share
            IMU-measured heading rate psi_dot (rad/s).
        observerCenterDistance : Share
            Output: estimated robot center distance traveled (m).
        observerHeading : Share
            Output: estimated heading angle from y_hat (rad).
        observerHeadingRate : Share
            Output: estimated heading rate from y_hat (rad/s).
        observerOmegaLeft : Share
            Output: estimated left-wheel angular velocity (rad/s).
        observerOmegaRight : Share
            Output: estimated right-wheel angular velocity (rad/s).
        observerDistanceLeft : Share
            Output: estimated left-wheel distance from y_hat (m).
        observerDistanceRight : Share
            Output: estimated right-wheel distance from y_hat (m).
        """

        self._goFlag = goFlag

        self.s_L = distLeft
        self.s_R = distRight
        self.u_L = voltageLeft
        self.u_R = voltageRight
        self.psi = heading
        self.psi_dot = headingRate

        # Reference module-level flat tuples — no conversion cost at instantiation.
        self._A = _A_D  # 4x4, length 16
        self._B = _B_D  # 4x6, length 24
        self._C = _C_D  # 4x4, length 16

        # State vectors as mutable float arrays — reused in place every cycle,
        # so no heap allocation occurs in the 50 Hz hot path.
        self.x_hat  = array('f', [0.0, 0.0, 0.0, 0.0])  # 4-element state
        self.y_hat  = array('f', [0.0, 0.0, 0.0, 0.0])  # 4-element output
        self._x_new = array('f', [0.0, 0.0, 0.0, 0.0])  # workspace for A@x+B@u
        self.u_aug  = array('f', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])  # 6-element input

        self._observerCenterDistance  = observerCenterDistance
        self._observerHeading         = observerHeading
        self._observerHeadingRate     = observerHeadingRate
        self._observerOmegaLeft       = observerOmegaLeft
        self._observerOmegaRight      = observerOmegaRight
        self._observerDistanceLeft    = observerDistanceLeft
        self._observerDistanceRight   = observerDistanceRight

        self._state = S0_IDLE

    @staticmethod
    def _update_x(A, B, x, u, out):
        """
        Propagate the state: out = A @ x + B @ u (zero heap allocation).

        A is a flat row-major 4x4 tuple, B is a flat row-major 4x6 tuple,
        x and u are ``array.array('f')`` of length 4 and 6 respectively,
        and out is a pre-allocated ``array.array('f')`` of length 4.
        """
        # Compute out = A(4x4)@x(4) + B(4x6)@u(6) with zero heap allocation.
        # A and B are flat tuples (row-major), x/u/out are array.array('f').
        for i in range(4):
            s = A[i*4]*x[0] + A[i*4+1]*x[1] + A[i*4+2]*x[2] + A[i*4+3]*x[3] \
              + B[i*6]*u[0] + B[i*6+1]*u[1] + B[i*6+2]*u[2] \
              + B[i*6+3]*u[3] + B[i*6+4]*u[4] + B[i*6+5]*u[5]
            out[i] = s

    @staticmethod
    def _update_y(C, x, out):
        """
        Compute the output estimate: out = C @ x (zero heap allocation).

        C is a flat row-major 4x4 tuple, x is ``array.array('f')`` of
        length 4, and out is a pre-allocated ``array.array('f')`` of length 4.
        """
        # Compute out = C(4x4)@x(4) with zero heap allocation.
        for i in range(4):
            out[i] = C[i*4]*x[0] + C[i*4+1]*x[1] + C[i*4+2]*x[2] + C[i*4+3]*x[3]

    def update(self, u_L, u_R, s_L, s_R, psi, psi_dot):
        """
        Advance the observer by one discrete time step.

        Reads sensor values from the caller, packs them into the reusable
        input buffer, propagates x_hat via A_D and B_D, then computes the
        output estimate y_hat via C_D.  All arithmetic uses pre-allocated
        buffers — no objects are created on the heap.

        Parameters
        ----------
        u_L : float  Left-motor voltage (V).
        u_R : float  Right-motor voltage (V).
        s_L : float  Left-wheel distance (m).
        s_R : float  Right-wheel distance (m).
        psi : float  IMU heading angle (rad).
        psi_dot : float  IMU heading rate (rad/s).
        """
        # Load inputs into reusable buffer.
        u = self.u_aug
        u[0] = u_L; u[1] = u_R; u[2] = s_L
        u[3] = s_R; u[4] = psi; u[5] = psi_dot

        # x_new = A @ x_hat + B @ u_aug  (zero allocation)
        self._update_x(self._A, self._B, self.x_hat, u, self._x_new)

        # Copy x_new into x_hat in place so the buffer objects never change.
        x = self.x_hat
        n = self._x_new
        x[0] = n[0]; x[1] = n[1]; x[2] = n[2]; x[3] = n[3]

        # y_hat = C @ x_hat  (zero allocation)
        self._update_y(self._C, self.x_hat, self.y_hat)

    def run(self):
        """
        Cooperative function for scheduler
        """
        while True:
            # Full Luenberger observer (state-machine version above) was
            # implemented and tested but replaced with the simple wheel-average
            # below for competition reliability — fewer failure modes and
            # identical straight-line accuracy in practice.
            # if self._state == S0_IDLE:
            #     if self._goFlag.get():
            #         self._state = S1_RUN

            # elif self._state == S1_RUN:
            #     if not self._goFlag.get():
            #         self._state = S0_IDLE
            #     else:
            #         self.update(
            #             self.u_L.get(),
            #             self.u_R.get(),
            #             self.s_L.get(),
            #             self.s_R.get(),
            #             self.psi.get(),
            #             self.psi_dot.get(),
            #         )
            #         # Publish: array.array element access avoids the float()
            #         # wrapper needed with ulab, keeping allocation minimal.
            #         x = self.x_hat
            #         y = self.y_hat
            #         self._observerCenterDistance.put(x[0])
            #         self._observerDistanceLeft.put(y[0])
            #         self._observerDistanceRight.put(y[1])
            #         self._observerHeading.put(y[2])
            #         self._observerHeadingRate.put(y[3])
            #         self._observerOmegaLeft.put(x[2])
            #         self._observerOmegaRight.put(x[3])
            # Simple fallback: center distance = average of both wheel odometers.
            self._observerCenterDistance.put(
                0.5 * (self.s_L.get() + self.s_R.get())
            )
            yield 0
