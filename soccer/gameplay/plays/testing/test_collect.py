import play
import behavior
import skills.collect
import robocup
import main


class TestCollect(play.Play):
    def __init__(self):
        super().__init__(continuous=True)
        self.add_transition(behavior.Behavior.State.start,
                            behavior.Behavior.State.running, lambda: True,
                            "immediately")

    def on_enter_running(self):
        self.add_subbehavior(
            skills.collect.Collect(), name='collect', required=True)
