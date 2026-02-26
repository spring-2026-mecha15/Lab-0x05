from pyb import Pin, Timer
from time import ticks_us, ticks_diff   # used for dt calculation in update()
from constants import *

UINT16_MAX = 0xFFFF

class Encoder:
    # Quadrature encoder interface wrapper.

    # - Uses a hardware Timer configured in ENC_AB mode with two channels
    #   (channel 1 -> A, channel 2 -> B).
    # - Tracks total accumulated encoder position accounting for timer
    #   reloads (overflow / underflow).
    # - Provides `update()` to sample the hardware counter and compute:
    #     - position: total encoder counts (signed, extended across reloads)
    #     - delta: change in position since last update (counts)
    #     - dt: elapsed time since last update (milliseconds, float)
    # - Convenience methods:
    #     - get_position() -> wheel linear position (mm)
    #     - get_velocity() -> linear velocity (mm/s)
    #     - zero() -> reset the encoder zero (tare)

    def __init__(self, tim, chA_pin, chB_pin):
        # Create and configure timer channels for quadrature decoding.

        # Parameters (comment-style hints, safe for MicroPython):
        #   tim     -- pyb.Timer instance already created (Timer(N))
        #   chA_pin -- pin identifier for channel A (connected to timer CH1)
        #   chB_pin -- pin identifier for channel B (connected to timer CH2)


        # Configure A/B pins as inputs
        pin_a = Pin(chA_pin, Pin.IN)
        pin_b = Pin(chB_pin, Pin.IN)

        # Hold reference to timer and initialize for encoder mode.
        # We set period to 0xFFFF to allow full 16-bit counting and register
        # a callback to detect reloads (overflow/underflow).
        self.timer = tim
        self.timer.init(period=UINT16_MAX, prescaler=0, callback=self._callback)

        # Attach timer channels for encoder A/B inputs
        self.timer.channel(1, pin=pin_a, mode=Timer.ENC_AB)
        self.timer.channel(2, pin=pin_b, mode=Timer.ENC_AB)

        # State variables
        self.position = 0                # Total accumulated position (extended counts)
        self.prev_count = 0              # Last recorded extended count (for delta)
        self.prev_ticks = ticks_us()     # Last timestamp in microseconds (0 initially)
        self.delta = 0                   # Change in extended counts since last update
        self.dt = 0                      # Elapsed time since last update (ms, float)

        # Timer reload bookkeeping:
        # increment when timer wrapped from 0xFFFF -> 0 (overflow)
        # decrement when timer wrapped from 0 -> 0xFFFF (underflow)
        self.timer_reloads = 0

    def _callback(self, timer_obj):
        # Timer callback invoked on timer reload.

        # The hardware timer invokes this callback when it reloads. We track
        # whether a reload represented an overflow or an underflow by sampling
        # the timer counter value and adjusting `self.timer_reloads` accordingly.

        # A try/except is used because, on some builds/hardware, reading the
        # timer inside the callback has occasionally raised unexpected errors.
        # The original code swallowed those exceptions; we preserve that behavior.

        try:
            cnt = timer_obj.counter()

            # Overflow: counter wrapped 0xFFFF -> 0 (observed value is 0)
            if cnt == 0:
                self.timer_reloads += 1

            # Underflow: counter wrapped 0 -> 0xFFFF (observed value is 0xFFFF)
            elif cnt == 0xFFFF:
                self.timer_reloads -= 1

        except Exception:
            # Preserve original behavior: ignore occasional read errors.
            # (TODO: investigate root cause separately)
            pass

    def update(self):
        # Sample the hardware counter and update position/delta/dt.

        # - Reads the raw 16-bit counter
        # - Extends it using `self.timer_reloads` to build a monotonic `position`
        # - Computes `delta = position - prev_count`
        # - Updates `prev_count` and computes elapsed time `dt` (ms)
        
        # Read raw 16-bit counter (0..65535)
        raw_count = self.timer.counter()

        # Extend the raw counter across reloads: position is raw_count +
        # UINT16_MAX * number_of_reloads. This preserves sign/monotonicity.
        self.position = raw_count + (UINT16_MAX * self.timer_reloads)

        # Delta (change in extended counts) since last update
        self.delta = self.position - self.prev_count

        # Save current extended count for next iteration
        self.prev_count = self.position

        # Time delta in milliseconds (float)
        now_us = ticks_us()
        self.dt = ticks_diff(now_us, self.prev_ticks) / 1000.0
        self.prev_ticks = now_us

    def get_position(self):
        # Convert the most recently-updated encoder `position` (counts) into
        # a linear wheel displacement in millimeters.

        # Formula:
        #     rotations = counts / ECOUNTS_PER_WREV
        #     distance = rotations * (2 * pi * wheel_radius)
        
        return (self.position / ECOUNTS_PER_WREV) * 2 * PI * WHEEL_RADIUS

    def get_velocity(self):
        # Compute linear velocity (mm/s) using the most recent `delta` and `dt`.

        # Returns 0 if dt is zero (to avoid division by zero). The computation
        # follows:
        #     counts_per_ms = delta / dt
        #     rotations_per_s = (counts_per_ms * 1000) / ECOUNTS_PER_WREV
        #     velocity_mm_s = rotations_per_s * (2 * pi * wheel_radius)
        
        if self.dt == 0:
            return 0

        ecounts_per_ms = self.delta / self.dt
        return (ecounts_per_ms * 1e3 / ECOUNTS_PER_WREV) * 2 * PI * WHEEL_RADIUS

    def zero(self):
        # Tare / zero the encoder.

        # - Reset reload counters and prev_count
        # - Reset the hardware timer counter to 0
        # - Call update() so subsequent reads are consistent with the new zero
        
        self.timer_reloads = 0
        self.prev_count = 0

        # Reset hardware counter
        self.timer.counter(0)

        # Refresh derived state so we don't return stale values immediately
        self.update()
