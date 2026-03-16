"""
task_motor.py

Cooperative task that wraps a PIController around a single Motor/Encoder pair
on the Romi robot.  Multiple instances are created (one per wheel) and run by
the cooperative scheduler.

States:
    S0_INIT (0): One-shot initialization; transitions immediately to S1_WAIT.
    S1_WAIT (1): Waits for the go flag to be set before enabling the motor.
    S2_RUN  (2): Runs closed-loop PI velocity control; transitions back to S1_WAIT when the go flag is cleared.
"""

from drivers.motor import Motor
from drivers.encoder import Encoder
from task_share   import Share, Queue
from utime        import ticks_us, ticks_diff
import micropython
from controller   import PIController
from constants    import *
try:
    import ujson as json
except ImportError:
    import json

S0_INIT = micropython.const(0) # State 0 - initialiation
S1_WAIT = micropython.const(1) # State 1 - wait for go command
S2_RUN  = micropython.const(2) # State 2 - run closed loop control


class task_motor:
    """
    Cooperative scheduler task for closed-loop velocity control of one motor.

    Each instance owns a Motor, an Encoder, and a PIController.  The task
    reads the encoder velocity, computes a PI correction, and drives the motor
    accordingly.  Instantiate once per wheel (left and right).
    """

    def __init__(self,
                 mot: Motor, enc: Encoder,
                 goFlag: Share, kpVal: Share, kiVal: Share, setpoint: Share,
                 dataValues: Queue, timeValues: Queue, wheelDistance: Share, motorVoltage: Share, motorAngVelocity: Share):
        """
        Initialize the motor task.

        Args:
            mot (Motor):                  Motor driver object used to set effort.
            enc (Encoder):                Encoder object used to read wheel
                                          position and velocity.
            goFlag (Share):               Integer flag; 0 = stop, 1 = run,
                                          2 = run with profiling data capture.
            kpVal (Share):                Proportional gain for the PI controller,
                                          readable and writable from the UI.
            kiVal (Share):                Integral gain for the PI controller,
                                          readable and writable from the UI.
            setpoint (Share):             Velocity setpoint (mm/s) written by the
                                          line-follow task or the UI.
            dataValues (Queue):           Queue for logging encoder velocity
                                          samples during profiling runs.
            timeValues (Queue):           Queue for logging timestamps (ms)
                                          corresponding to each velocity sample.
            wheelDistance (Share):        Cumulative wheel travel (encoder counts)
                                          reported to the observer task.
            motorVoltage (Share):         Commanded motor voltage (V) reported for
                                          telemetry and the UI.
            motorAngVelocity (Share):     Wheel angular velocity (rad/s) derived
                                          from the encoder and reported for
                                          telemetry.
        """


        self._state: int        = S0_INIT    # The present state of the task       
        
        self._mot: Motor        = mot        # A motor object
        self._enc: Encoder      = enc        # An encoder object

        self._goFlag: Share     = goFlag     # A share object representing a
                                             # flag to start data collection

        self._kpVal: Share      = kpVal      # A share for Kp gain from user
                                             # interface
        self._kiVal: Share      = kiVal      # A share for Ki gain from user
                                             # interface
        self._setpoint: Share   = setpoint   # A share for setpoint value
                                             # from user interface

        self._wheelDistance: Share = wheelDistance  #A share of distance traveled by the wheel from observer
        
        self._motorVoltage: Share = motorVoltage  # A share of commanded motor voltage [V]

        self._motorOmega: Share = motorAngVelocity  # A share of commanded motor angular velocity [V]

        self._dataValues: Queue = dataValues # A queue object used to store
                                             # collected encoder position
        self._timeValues: Queue = timeValues # A queue object used to store the
                                             # time stamps associated with the
                                             # collected encoder data
        self._startTime: int    = 0          # The start time (in microseconds)
                                             # for a batch of collected data

        self._controller = PIController(
            self._apply_effort,     # Actuator callback ()
            MOTOR_GAIN,                   # Actuator gain
            self._enc.get_velocity,   # Sensor  (encoder counts per ms)
            # ENCODER_GAIN,                   # Sensor gain
            1,
            (-100, 100)               # Saturation min, max range
                                      # For motor: -100 to 100 % duty cycle
        )

        self._load_gains()

        # Cache initial gains in memory
        self._kp_init = self._kpVal.get()
        self._ki_init = self._kiVal.get()
        self._controller.Kp = self._kp_init
        self._controller.Ki = self._ki_init

        self._profiling = False
        
        print("Motor Task object instantiated")

    def _apply_effort(self, effort_pct):
        """
        Actuator callback invoked by PIController with the computed effort.

        Converts the percent effort to an equivalent voltage assuming a 6.5 V
        supply, writes the result to the motorVoltage share for telemetry, then
        forwards the raw percent value to the motor driver.

        Args:
            effort_pct (float): Desired motor effort in percent (-100 to 100).
        """
        motor_voltage = (effort_pct / 100.0) * 6.5
        self._motorVoltage.put(motor_voltage)
        
        self._mot.set_effort(effort_pct)

    def _load_gains(self) -> bool:
        """
        Read motor PI gains from the JSON gains file.

        Attempts to open GAINS_FILE and parse the ``motor`` section.  If a
        value for ``kp`` or ``ki`` is present it is written to the corresponding
        Share; otherwise the compile-time default constant is used instead.

        Returns:
            bool: True if the file was read and parsed successfully, False if the
                  file could not be opened or contained invalid JSON.
        """
        try:
            with open(GAINS_FILE, "r") as gains_file:
                data = json.load(gains_file)
        except (OSError, ValueError):
            return False

        motor = data.get("motor", {})
        motor_kp = motor.get("kp")
        motor_ki = motor.get("ki")

        if motor_kp is not None:
            self._kpVal.put(float(motor_kp))
            print(f"Read motor Kp: {float(motor_kp)}")
        else:
            self._kpVal.put()
            print(f"Motor Kp not found. Using default: {DEFAULT_MOTOR_KP}")
        if motor_ki is not None:
            self._kiVal.put(float(motor_ki))
            print(f"Read motor Ki: {float(motor_ki)}")
        else:
            self._kiVal.put()
            print(f"Motor Ki not found. Using default: {DEFAULT_MOTOR_KI}")

        return True
        
    def run(self):
        """
        Cooperative function for scheduler
        """
        while True:
            
            if self._state == S0_INIT: # Init state (can be removed if unneeded)
                # print("Initializing motor task")
                self._state = S1_WAIT
                
            elif self._state == S1_WAIT: # Wait for "go command" state
                go = self._goFlag.get()

                if go == 2:
                    self._profiling = True
                else:
                    self._profiling = False

                if go:
                    # print("Starting motor loop")
                    
                    # Capture a start time in microseconds so that each sample
                    # can be timestamped with respect to this start time. The
                    # start time will be off by however long it takes to
                    # transition and run the next state, so the time values may
                    # need to be zeroed out again during data processing.
                    self._startTime = ticks_us()
                    self._state = S2_RUN
                    self._mot.enable()
                    self._controller.reset()
                    self._enc.zero()
                    self._controller.set_point = self._setpoint.get()
                    self._controller.Kp = self._kpVal.get()
                    self._controller.Ki = self._kiVal.get()


                
            elif self._state == S2_RUN: # Closed-loop control state
                # print(f"Running motor loop, cycle {self._dataValues.num_in()}")

                # Check if we need to exit motor control
                if not self._goFlag.get():
                    self._state = S1_WAIT
                    self._mot.disable()
                    self._enc.zero()

                self._controller.set_point = self._setpoint.get()
                
                # Run the encoder update algorithm and then capture the present
                # position of the encoder. You will eventually need to capture
                # the motor speed instead of position here.
                # self._enc.update()
                
                # Collect a timestamp to use for this sample
                t   = ticks_us()
                
                # Actuate the motor using a control law. The one used here in
                # the example is a "bang bang" controller, and will work very
                # poorly in practice. Note that the set position is zero. You
                # will replace this with the output of your PID controller that
                # uses feedback from the velocity measurement.
                # self._mot.set_effort(30 if pos < 0 else -30)
                # Update encoder before measuring velocity
                self._enc.update()
                self._controller.run()
                self._wheelDistance.put(self._enc.get_position())

                #Report Motor Angular Velocity
                omega = self._enc.get_velocity() / WHEEL_RADIUS
                self._motorOmega.put(omega)                 # Store angular to be reported to output


                if self._profiling:
                    vel = self._enc.get_velocity()

                    # # Store the sampled values in the queues
                    # # self._dataValues.put(pos)
                    self._dataValues.put(vel)                                   # Store velocity to be reported to output
                    self._timeValues.put(int(ticks_diff(t, self._startTime) / 1000)) # Convert from uS to mS (10^3)


            
            yield self._state
