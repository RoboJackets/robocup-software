import main
import robocup
import behavior
import constants

import standard_play
import skills.line_kick
import tactics.coordinated_pass

class BasicIndirect(standard_play.StandardPlay):

    def __init__(self, indirect=None):
        super().__init__(continuous=True)

        self.add_transition(behavior.Behavior.State.start,
                            behavior.Behavior.State.running, lambda: True,
                            'immediately')

        kicker = skills.line_kick.LineKick()
        kicker.use_chipper = True

        kicker.kick_power = 50

        receive_pt = robocup.Point(0, 3 * constants.Field.Length / 4)
        
        pass_behavior = tactics.coordinated_pass.CoordinatedPass(
            receive_pt,
            None,
            (kicker, lambda x: True),
            receiver_required=False,
            kicker_required=False,
            prekick_timeout=9)
        self.add_subbehavior(pass_behavior, 'pass')

        self.add_transition(
            behavior.Behavior.State.running, behavior.Behavior.State.completed,
            lambda: self.subbehavior_with_name('pass').is_done_running() and self.subbehavior_with_name('pass').state != tactics.coordinated_pass.CoordinatedPass.State.timeout,
            'pass completes')

    @classmethod
    def score(cls):
        gs = main.game_state()
        return 0 if behavior.Behavior.State.running or (
            gs.is_ready_state() and gs.is_our_indirect_kick()) else float("inf")

    @classmethod
    def is_restart(cls):
        return True
