import play
import behavior
import tactics.positions.defender
import robocup
import main


class TestDefender(play.Play):
    def __init__(self):
        super().__init__(continuous=True)
        self.add_transition(behavior.Behavior.State.start,
                            behavior.Behavior.State.running, lambda: True,
                            "immediately")

    def on_enter_running(self):
        b = tactics.positions.defender.Defender(
            tactics.positions.defender.Defender.Side.left)
        self.add_subbehavior(b, name='defender', required=True)

    def on_exit_running(self):
        self.remove_subbehavior('defender')
