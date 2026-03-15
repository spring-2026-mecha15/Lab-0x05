from pyb import Pin, Timer, ADC
import gc


class Motor:
    """
    Simple motor driver wrapper for a Romi-style motor driver.

    Responsibilities:
      - configure a PWM channel for the motor
      - control the direction pin
      - control the sleep (enable) pin
      - provide a simple set_effort(value) interface where value is in
        -100..100 (percent). Positive values select one direction,
        negative values select the other direction.

    Notes:
      - `pwm`/`slp`/`dir` arguments may be either Pin instances or arguments
        that the pyb.Pin constructor accepts
    """
    

    def __init__(self, pwm, tim, channel, slp, dir, battAdc):
        """
        Initialize motor control.

        Parameters
          pwm      -- pin identifier / pyb.Pin for PWM output (passed to Timer.channel)
          tim      -- pyb.Timer instance used for PWM
          channel  -- integer channel number for the timer (1..4 typically)
          slp      -- pin identifier / pyb.Pin for sleep (enable) pin
          dir      -- pin identifier / pyb.Pin for direction pin
          batt_adc -- pin identifier / pyb.Pin for battery ADC
        """

        # Create/configure the timer channel for PWM (starts at 0% duty).
        # This mirrors the original usage of tim.channel(..., pulse_width_percent=0).
        self.pwm = tim.channel(channel, pin=pwm, mode=Timer.PWM, pulse_width_percent=0)

        # Configure direction and sleep pins as push-pull outputs.
        # Note: original code used Pin(slp, Pin.OUT_PP) etc., so we preserve that.
        self.sleep = Pin(slp, Pin.OUT_PP)
        self.dir = Pin(dir, Pin.OUT_PP)

        # Configure battery ADC pin for battery droop compensation
        self._battAdc = ADC(Pin(battAdc))

        # Start disabled and with zero effort to match original behavior.
        self.disable()
        self.set_effort(0)

    def enable(self):
        """Enable the motor driver by setting the SLEEP pin high."""

        self.sleep.high()

    def disable(self):
        """
        Disable the motor driver by setting the SLEEP pin low.

        The original implementation also set effort to zero when disabling;
        we preserve that behavior by calling set_effort(0) first.
        """
        
        # Reset PWM to zero effort before disabling (preserves original behavior).
        self.set_effort(0)
        self.sleep.low()

    def set_effort(self, value):
        """
        Set motor effort.

        `value` is interpreted as percent in the range -100..100 (as in the
        original code). Positive selects one direction, negative the opposite.
        The method writes the direction pin and then sets PWM duty to the
        absolute magnitude of `value` (via pulse_width_percent).

        The effort is capped such that 100% effort corresponds to 6.5V by
        measuring battery voltage and applying a scale factor to the
        effort requested

        IMPORTANT: behavior unchanged from original code — there is no extra
        validation/clamping here.
        """
        
        # Choose direction pin state:
        # - original code sets dir.low() when value > 0, else dir.high()
        if value > 0:
            self.dir.low()
        else:
            self.dir.high()

        # --- Clip effort to valid range (-100 to 100) ---
        if value > 100:
            value = 100
        elif value < -100:
            value = -100

        # --- Scale value to account for battery droop
        adcVoltage = self._battAdc.read() / 4096 * 3.3

        # Scale for 4.7k and 10k voltage divider.
        # (Slightly tweaked to account for actual VDD)
        battVoltage = adcVoltage / 0.305

        # Calculate scaling factor
        effortScale = 6.5 / battVoltage

        # Apply scaling factor to battery voltage
        value *= effortScale
            
        # Apply absolute percent duty to PWM channel.
        self.pwm.pulse_width_percent(abs(value))

gc.collect()