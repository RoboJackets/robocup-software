import play
import behavior
import tactics.defense_rewrite
import robocup
import main


## Runs our Defense tactic
class TestDefenseRewrite(play.Play):
    def __init__(self):
        super().__init__(continuous=True)
        self.add_transition(behavior.Behavior.State.start,
                            behavior.Behavior.State.running, lambda: True,
                            "immediately")

    def on_enter_running(self):
        b = tactics.defense_rewrite.DefenseRewrite()
        self.add_subbehavior(b, name='defense', required=True)

    def on_exit_running(self):
        self.remove_subbehavior('defense')

    @classmethod
    def handles_goalie(cls):
        return True
