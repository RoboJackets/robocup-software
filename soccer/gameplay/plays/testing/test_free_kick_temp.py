import play
import behavior
import robocup
import main
import evaluation.shooting

class TestFreeKickTemp(play.Play):
    def __init__(self):
        super().__init__(continuous=True)
        self.add_transition(behavior.Behavior.State.start,
                            behavior.Behavior.State.running, lambda: True,
                            "immediately")

    def execute_running(self):
        evaluation.shooting.find_gap()