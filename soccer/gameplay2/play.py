from behavior import *


class Play(Behavior):

    def __init__(self, continuous):
        super().__init__(continuous)
        self._robots = None
        

    @property
    def robots(self):
        return self._robots
    @robots.setter
    def robots(self, value):
        self._robots = value
    