import play
import behavior
import skills.fullback
import robocup
import main

class TestFullback(play.Play):

    def __init__(self):
        super().__init__(continuous=True)
        self.add_transition(behavior.Behavior.State.start, behavior.Behavior.State.running, lambda: True, "immediately")

    def on_enter_running(self):
    	b = skills.fullback.Fullback(skills.fullback.Fullback.Side.left)
    	self.add_subbehavior(b, name='fullback', required=True)

    def on_exit_running(self):
        self.remove_subbehavior('fullback')
