from behavior import *


# a skill is a behavior that pertains to a SINGLE robot
class Skill(Behavior):

    @property
    def robot(self):
        return self._robot
    @robot.setter
    def robot(self, value):
        self._robot = value
