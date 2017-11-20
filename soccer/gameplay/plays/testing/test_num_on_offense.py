import play
import behavior
import evaluation.opponent
import constants
import robocup


class TestNumOnOffense(play.Play):
    def __init__(self):
        super().__init__(continuous=True)

        self.add_transition(behavior.Behavior.State.start,
                            behavior.Behavior.State.running, lambda: True,
                            'immediately')

    def execute_running(self):
        print(evaluation.opponent.num_on_offense())
