from behavior import *


class Play(Behavior):

    def __init__(self, continuous):
        super().__init__(continuous)
        self._robots = None


    # TODO: clarify what this does and what the values mean.  is higher or lower more likely to get picked?
    @classmethod
    def score(cls):
        return 0
        

    @property
    def robots(self):
        return self._robots
    @robots.setter
    def robots(self, value):
        self._robots = value
    