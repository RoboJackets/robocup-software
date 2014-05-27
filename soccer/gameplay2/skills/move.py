from skill import *
from behavior import *


# wraps up OurRobot.move() into a Skill so we can use it in the play system more easily
class Move(Skill):

    def __init__(self, pos=None):
        super().__init__(continuous=False)

        self.threshold = 0.05
        self.pos = pos

        self.add_transition(Behavior.State.start, Behavior.State.running, lambda: True, 'immediately')
        self.add_transition(Behavior.State.running, Behavior.State.completed,
            lambda:
                self.robot != None and
                self.pos != None and
                (self.robot.pos - self.pos).mag() < self.threshold,
            'target pos reached'
            )


    # the position to move to
    @property
    def pos(self):
        return self._pos
    @pos.setter
    def pos(self, value):
        self._pos = value


    # how close (in meters) the robot has to be to the target position for it be complete
    @property
    def threshold(self):
        return self._threshold
    @threshold.setter
    def threshold(self, value):
        self._threshold = value
