import standard_play
import behavior
import tactics.defense
import robocup
import main


## Runs our Defense tactic
class TestDefense(standard_play.StandardPlay):
    def __init__(self):
        super().__init__(continuous=True)
        self.add_transition(behavior.Behavior.State.start,
                            behavior.Behavior.State.running, lambda: True,
                            "immediately")

    @classmethod
    def handles_goalie(cls):
        return True
