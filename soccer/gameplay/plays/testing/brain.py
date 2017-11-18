import play
import tactics.line_up
import behavior_sequence
import tools.sleep
import robocup
import constants
import time
import enum

class Brain(play.Play):

    # initialize constants, etc.
    def __init__(self):
        # not sure if we need this
        super().__init__(continuous=True)

    class State(enum.Enum):
        waiting = 0
        dummy = 0

