import play
import behavior
import tactics.defense2
import robocup
import main


## Runs our Defense tactic
class TestDefense2(play.Play):
    def __init__(self):
        super().__init__(continuous=True)
        self.add_transition(behavior.Behavior.State.start,
                            behavior.Behavior.State.running, lambda: True,
                            "immediately")

    def on_enter_running(self):
        b = tactics.defense2.Defense2()
        self.add_subbehavior(b, name='defense', required=True)

    def on_exit_running(self):
        self.remove_subbehavior('defense')

    @classmethod
    def handles_goalie(cls):
        return True
