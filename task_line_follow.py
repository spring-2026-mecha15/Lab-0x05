from task_share import Share
from controller import PIController

SPEED = 25

class task_line_follow:
    def __init__(
            self,
            lineFollowGo:       Share,
            lineCentroid:       Share,
            rightMotorSetPoint: Share,
            leftMotorSetPoint:  Share
        ):
        
        self._state = 0

        self._lineFollowGo = lineFollowGo

        self._lineCentroid = lineCentroid

        self._rightMotorSetPoint = rightMotorSetPoint

        self._leftMotorSetPoint = leftMotorSetPoint

        self._controller = PIController(
            self.plant,
            1,
            self._lineCentroid.get,
            1,
            (-100, 100)
        )

        # Set Controller Parameters
        self._controller.Ki = 1                  # Will implement integral control

        self._controller.Kp = 25                  # Proportional gain

        self._controller.set_point = 0           # Setpoint is 0: corresponding to 
                                                 # keeping the line centered under romi

        # self._count = 0

        print("Line follower task instantiated")

    def run(self):
        while True:
            if self._lineFollowGo.get():
                self._controller.run()

            yield self._state

    def plant(self, value):
        # print(f'{self._count} - Plant rx value: {value:.3f}')
        # self._count += 1
        self._leftMotorSetPoint.put(SPEED + value)
        self._rightMotorSetPoint.put(SPEED - value)
