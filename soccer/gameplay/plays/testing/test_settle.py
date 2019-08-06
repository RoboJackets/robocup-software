import play
import behavior
import skills.settle
import robocup
import main


class TestSettle(play.Play):
    def __init__(self):
        super().__init__(continuous=True)
        self.add_transition(behavior.Behavior.State.start,
                            behavior.Behavior.State.running, lambda: True,
                            "immediately")

    def on_enter_running(self):
        self.add_subbehavior(
            skills.settle.Settle(), name='settler', required=True)
