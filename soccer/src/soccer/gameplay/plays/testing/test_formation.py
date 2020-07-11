import play
import behavior
import formation.controller
import robocup
import main


class TestFormation(play.Play):
    def __init__(self):
        super().__init__(continuous=True)
        self.add_transition(behavior.Behavior.State.start,
                            behavior.Behavior.State.running, lambda: True,
                            "immediately")

    def on_enter_running(self):
        c = formation.controller.Controller()
        self.add_subbehavior(c, name='controller', required=True)

    def on_exit_running(self):
        self.remove_subbehavior('controller')
