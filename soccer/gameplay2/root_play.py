from play import *
from behavior import *
import logging


# the RootPlay is basically the python-side of the c++ GameplayModule
# it coordinates the selection of the 'actual' play and handles the goalie behavior
class RootPlay(Play):

    def __init__(self):
        super().__init__(continuous=True)
        self._play = None

        self.add_transition(Behavior.State.start, Behavior.State.running, lambda: True, 'immediately')


    def execute_running(self):
        logging.info("RootPlay running...")
        pass


    @property
    def play(self):
        return self._play


    @property
    def goalie_behavior(self):
        return self._goalie_behavior
