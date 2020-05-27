import composite_behavior
from typing import List
from situations import Situation
import main
import random

## @brief A play coordinates the entire team of robots
# @details Only play runs at a time.
# By default, the RootPlay (which every Play is a subbehavior of) handles the Goalie,
# however, by overriding the handles_goalie() class method, a Play can choose to handle
# the goalie on its own, which allows for greater coordination.
class Play(composite_behavior.CompositeBehavior):
    def __init__(self, continuous: bool):
        super().__init__(continuous)

    ## Used to determine when to run a play
    # Return float("inf") if the play cannot be used or a score (lower is better) used to select the best play.
    @classmethod
    def score(cls) -> float:
        return 10

    _situationList: List[Situation] = list()

    @classmethod
    def is_restart(cls) -> bool:
        return False

    ## Override to allow a play to run during the stopped state
    # This is reserved only for very special plays that need to run during the
    # stopped state.  Most plays should give up control during the stopped state
    # to stopped.py, so stopped.py is the only play that should be overriding
    # this.
    @classmethod
    def run_during_stopped(cls) -> bool:
        return False

    ##
    # Call to attempt to preempt the play
    # Returns true if the preempt is successful
    # Override if you want more complex responce to being preempted
    def try_preempt(self) -> bool:
        self.terminate()
        return True

    ##
    # Returns true if this play is valid for the passed situation
    #
    @classmethod
    def is_valid(cls, situation: Situation) -> bool:
        return situation in cls._situationList

    ##
    # The score function for standard play will check if situation analysis
    # is enabled, will return a base score based on situation match if it is
    # or return float('inf') if it is not
    #
    # The expected interaction is to override this function can call super()
    # to get a baseline score, then to modify that score based on situational
    # factors. If you get a float("inf") back you know that situation analysis
    # is not running and you will need to overwrite that with some other score
    #
    @classmethod
    def score(cls) -> float:
        if not main.situationAnalysis.enabled:
            return float('inf')
        else:
            if cls.is_valid(main.situationAnalysis.getSituation()):
                return main.situationAnalysis.inSituationScore + random.random(
                )
            else:
                return main.situationAnalysis.outSituationScore + random.random(
                )
