import play
import behavior
import skills.bump
import robocup
import main


class TestBump(play.Play):

    def __init__(self):
        super().__init__(continuous=True)
        self.add_transition(behavior.Behavior.State.start, behavior.Behavior.State.running, lambda: True, "immediately")

    def on_enter_running(self):
        b = skills.bump.Bump()
        self.add_subbehavior(b, name='bump', required=True)

    def on_exit_running(self):
        self.remove_subbehavior('bump')
