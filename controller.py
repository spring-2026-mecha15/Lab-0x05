from utime import ticks_us, ticks_diff


class PIController:
    # Generic PI (Proportional + Integral) controller for MicroPython.

    # Each `run()` call:
    #     1) reads sensor
    #     2) computes error and dt
    #     3) updates integral (with anti-windup)
    #     4) computes PI output
    #     5) clamps to saturation
    #     6) sends to actuator
    

    def __init__(
        self,
        actuator_cb,                  # Callable[[float], None]
        actuator_gain: float,         # gain applied to actuator effort
        sensor_cb,                    # Callable[[], float]
        sensor_gain: float,           # gain applied to sensor measurement
        saturation_range: tuple[float, float],
    ):
        # External I/O
        self._actuator_cb = actuator_cb
        self._sensor_cb = sensor_cb

        # Gains
        self._actuator_gain: float = actuator_gain
        self._sensor_gain: float = sensor_gain

        # Saturation limits
        self._sat_min, self._sat_max = saturation_range

        # Controller gains (defaults preserved as in original)
        self._Kp = 0.0
        self._Ki = 0.0
        self._Kff = 0.0

        # Controller state
        self._setpoint = 0.0
        self._ff_setpoint = 0.0
        self._last_ticksus = 0
        self._last_measured = 0.0
        self._i_error = 0.0

        # Initialize timing and integrator state
        self.reset()

    # --------------------
    # Public properties
    # --------------------
    @property
    def Kp(self) -> float:
        return self._Kp

    @Kp.setter
    def Kp(self, value: float) -> None:
        self._Kp = value

    @property
    def Ki(self) -> float:
        return self._Ki

    @Ki.setter
    def Ki(self, value: float) -> None:
        self._Ki = value

    @property
    def set_point(self) -> float:
        return self._setpoint

    @set_point.setter
    def set_point(self, value: float) -> None:
        self._setpoint = value

    def set_feed_forward(self, setpoint: float, gain: float) -> None:
        """
        Set feed-forward setpoint and gain.

        Feed-forward effort is computed as:
            ff_term = gain * setpoint
        and is added to the PI effort before saturation.
        """
        self._ff_setpoint = setpoint
        self._Kff = gain

    # --------------------
    # Control methods
    # --------------------
    def reset(self) -> None:
        """
        Reset integral state and initialize the time reference.
        """
        self._i_error = 0.0
        self._last_ticksus = ticks_us()

    def run(self) -> None:
        """
        Run one PI update with anti-windup protection.
        """

        # --- Time delta (wrap-safe) ---
        now_us = ticks_us()
        dt_s = ticks_diff(now_us, self._last_ticksus) * 1e-6  # µs -> s
        if dt_s < 0:
            dt_s = 0.0
        self._last_ticksus = now_us

        # --- Measurement ---
        measured = self._sensor_cb() * self._sensor_gain
        self._last_measured = measured

        # --- Error and proportional term ---
        error = self._setpoint - measured
        p_term = self._Kp * error

        # --- Anti-windup: predict integrator update, decide whether to accept it ---
        # Candidate integrator after this timestep
        candidate_i_error = self._i_error + (error * dt_s)
        candidate_i_term = self._Ki * candidate_i_error

        # Current PI pre-saturation using current integrator
        current_i_term = self._Ki * self._i_error
        current_pre_sat = p_term + current_i_term
        candidate_pre_sat = p_term + candidate_i_term

        # Decide whether to accept candidate integrator:
        # - Accept if candidate output would be within saturation limits, OR
        # - Accept if current output is saturated but the current error would move
        #   the output back towards the allowed range (helps recovery)
        accept_integration = False
        if self._sat_min <= candidate_pre_sat <= self._sat_max:
            accept_integration = True
        else:
            # If currently above max and error is negative, integration will reduce output -> accept
            if (current_pre_sat > self._sat_max) and (error < 0):
                accept_integration = True
            # If currently below min and error is positive, integration will increase output -> accept
            if (current_pre_sat < self._sat_min) and (error > 0):
                accept_integration = True

        if accept_integration:
            # Commit the integrator update
            self._i_error = candidate_i_error
            i_term = candidate_i_term
        else:
            # Do not change integrator (prevents wind-up)
            i_term = current_i_term

        # --- Compute final effort and saturate ---
        ff_term = self._Kff * self._ff_setpoint
        effort_pre_sat = p_term + i_term + ff_term
        effort_pre_sat *= self._actuator_gain

        if effort_pre_sat < self._sat_min:
            effort = self._sat_min
        elif effort_pre_sat > self._sat_max:
            effort = self._sat_max
        else:
            effort = effort_pre_sat

        # --- Send to actuator ---
        self._actuator_cb(effort)

        # Optional debug (commented)
        # print("err:", error, "dt:", dt_s, "I:", self._i_error, "out:", effort)
