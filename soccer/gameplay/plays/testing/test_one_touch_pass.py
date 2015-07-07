import play
import behavior
import tactics.one_touch_pass
import tactics.behavior_sequence
import robocup
import constants
import main


## Continually runs a one_touch_pass pass tactic
class TestOneTouchPass(play.Play):

    def __init__(self):
        super().__init__(continuous=True)

        self.add_transition(behavior.Behavior.State.start,
            behavior.Behavior.State.running,
            lambda: True,
            'immediately')

        pass_bhvr = tactics.one_touch_pass.OneTouchPass()
        self.add_subbehavior(pass_bhvr, 'pass')

    def execute_running(self):
        pass_bhvr = self.subbehavior_with_name('pass')

        if pass_bhvr.is_done_running():
            self.remove_subbehavior('pass')
            pass_bhvr = tactics.one_touch_pass.OneTouchPass()
            self.add_subbehavior(pass_bhvr, 'pass')

