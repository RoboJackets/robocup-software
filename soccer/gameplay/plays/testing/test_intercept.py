import play
import behavior
import skills.intercept
import robocup
import main


class TestIntercept(play.Play):

    def __init__(self):
        super().__init__(continuous=True)
        self.add_transition(behavior.Behavior.State.start, behavior.Behavior.State.running, lambda: True, "immediately")

    def on_enter_running(self):
    	self.add_subbehavior(skills.intercept.Intercept(), name='intercept', required=True)

    def on_exit_running(self):
        self.remove_subbehavior('intercept')
