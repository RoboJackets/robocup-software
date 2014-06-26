import play
import behavior
import tactics.circle_up
import robocup
import main

class TestIdle(play.Play):

    def __init__(self):
        super().__init__(continuous=True)
        self.add_transition(behavior.Behavior.State.start, behavior.Behavior.State.running, lambda: True, "immediately")

    def on_enter_running(self):
            self.add_subbehavior(tactics.circle_up.CircleUp(), name='CircleUp', required=True)

    def on_exit_running(self):
        self.remove_subbehavior('CircleUp')
