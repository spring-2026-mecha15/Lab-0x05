from micropython import const
from task_share import Share

S0 = const(0)     # Idle task. Wait for go command
S1 = const(1)     # Run from point 0-1 (line follow with diverge)
S2 = const(2)     # Run from point 1-2 (line follow with 180 curve)
S3 = const(3)     # Run from point 2-3 (line follow with intermittent line)
S4 = const(4)     # Run from point 3-4 (line follow on "square wave")
S5 = const(5)     # Run from point 4-5 (parking garage)

class task_competition:
    def __init__(self, goFlag: Share):
        
        self._goFlag = goFlag

        self._state = S0

    def run(self):
        while True:
            if self._state == S0:
                if self._goFlag.get():
                    self._state = S1

            elif self._state == S1:
                pass

            elif self._state == S2:
                pass

            elif self._state == S3:
                pass

            elif self._state == S4:
                pass

            elif self._state == S5:
                pass

            yield