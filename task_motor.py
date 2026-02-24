''' This file demonstrates an example motor task using a custom class with a
    run method implemented as a generator
'''
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
    '''
    A class that represents a motor task. The task is responsible for reading
    data from an encoder, performing closed loop control, and actuating a motor.
    Multiple objects of this class can be created to work with multiple motors
    and encoders.
    '''

    def __init__(self,
                 mot: Motor, enc: Encoder,
                 goFlag: Share, kpVal: Share, kiVal: Share, setpoint: Share, dataValues: Queue, timeValues: Queue):
        '''
        Initializes a motor task object
        
        Args:
            mot (motor_driver): A motor driver object
            enc (encoder):      An encoder object
            goFlag (Share):     A share object representing a boolean flag to
                                start data collection
            dataValues (Queue): A queue object used to store collected encoder
                                position values
            timeValues (Queue): A queue object used to store the time stamps
                                associated with the collected encoder data
        '''

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
        
        self._dataValues: Queue = dataValues # A queue object used to store
                                             # collected encoder position
        
        self._timeValues: Queue = timeValues # A queue object used to store the
                                             # time stamps associated with the
                                             # collected encoder data
        
        self._startTime: int    = 0          # The start time (in microseconds)
                                             # for a batch of collected data

        self._controller = PIController(
            self._mot.set_effort,     # Actuator callback ()
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
        
        print("Motor Task object instantiated")

    def _load_gains(self) -> bool:
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
        if motor_ki is not None:
            self._kiVal.put(float(motor_ki))
            print(f"Read motor Ki: {float(motor_ki)}")

        return True
        
    def run(self):
        '''
        Runs one iteration of the task
        '''
        
        while True:
            
            if self._state == S0_INIT: # Init state (can be removed if unneeded)
                # print("Initializing motor task")
                self._state = S1_WAIT
                
            elif self._state == S1_WAIT: # Wait for "go command" state
                if self._goFlag.get():
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
                if not(self._goFlag.get()):
                    self._state = S1_WAIT
                    self._mot.disable()

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
                """
                # pos = self._enc.get_position()
                vel = self._enc.get_velocity()

                # print('position: ', pos, ', delta:', self._enc.delta, ', dt:', self._enc.dt, ', vel:', vel)
                
                # Store the sampled values in the queues
                # self._dataValues.put(pos)
                self._dataValues.put(vel)                                   # Store velocity to be reported to output
                self._timeValues.put(int(ticks_diff(t, self._startTime) / 1000)) # Convert from uS to mS (10^3)
                
                # When the queues are full, data collection is over
                if self._dataValues.full():
                    # print("Exiting motor loop")
                    self._state = S1_WAIT
                    self._goFlag.put(False)
                    self._mot.disable()
                """


            
            yield self._state
