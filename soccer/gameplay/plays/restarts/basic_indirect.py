import standard_play
import behavior
import skills.move
import skills.pivot_kick
import constants
import robocup
import math
import main
import tactics.coordinated_pass
import evaluation.passing_positioning
import enum


class BasicIndirect(standard_play.StandardPlay):

    def __init__(self, indirect=None):
        super().__init__(continuous=True)

        self.add_transition(behavior.Behavior.State.start,
                            behavior.Behavior.State.running, lambda: True,
                            'immediately')

        # FIXME: this could also be a PivotKick
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
        self.add_subbehavior(pass_behavior, 'kicker')

        self.add_transition(
            behavior.Behavior.State.running, behavior.Behavior.State.completed,
            lambda: self.subbehavior_with_name('kicker').is_done_running() and self.subbehavior_with_name('kicker').state != tactics.coordinated_pass.CoordinatedPass.State.timeout,
            'kicker completes')

    @classmethod
    def score(cls):
        gs = main.game_state()
        return 0 if behavior.Behavior.State.running or (
            gs.is_ready_state() and gs.is_our_free_kick()) else float("inf")

    @classmethod
    def is_restart(cls):
        return True
