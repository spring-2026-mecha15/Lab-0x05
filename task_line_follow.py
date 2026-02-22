from task_share import Share
from controller import PIController


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

        self.con = PIController(
            self.plant,
            1,
            self._lineCentroid.get,
            1,
            (-100, 100)
        )

        print("Line follower task instantiated")

    def run(self):
        while True:
            if self._lineFollowGo.get():
                self.con.set_point = 0
                self.con.run()

            yield self._state

    def plant(self, value):
        print(f'Received plant value from PI controller: {value:.3f}')