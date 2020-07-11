import main
import robocup
import behavior
import constants

import standard_play
import skills.line_kick
import tactics.coordinated_pass
import enum
import skills.move

class BasicIndirect(standard_play.StandardPlay):

    class State(enum.Enum):

        move = 1

        kick = 2

    def __init__(self, indirect=None):
        super().__init__(continuous=True)

        for s in BasicIndirect.State:
            self.add_state(s, behavior.Behavior.State.running)

        self.add_transition(behavior.Behavior.State.start,
                            BasicIndirect.State.move, lambda: True,
                            'immediately')

        self.add_transition(
            BasicIndirect.State.move,
            BasicIndirect.State.kick, lambda: self.subbehavior_with_name(
                'move').state == behavior.Behavior.State.completed, 'kick')

        self.add_transition(
            BasicIndirect.State.kick,
            behavior.Behavior.State.completed,
            lambda: self.subbehavior_with_name('pass').is_done_running(
            ) and self.subbehavior_with_name('pass').state != tactics.
            coordinated_pass.CoordinatedPass.State.timeout,
            # Keep trying pass until timeout
            'pass completes')

    @classmethod
    def score(cls):
        gs = main.game_state()
        return 0 if behavior.Behavior.State.running or (
            gs.is_ready_state() and
            gs.is_our_indirect_kick()) else float("inf")

    @classmethod
    def is_restart(cls):
        return True

    def on_enter_move(self):
        self.move_to = self.calc_move_pt()
        self.add_subbehavior(
            skills.move.Move(self.move_to), 'move', required=False, priority=5)

    def execute_move(self):
        self.move_to = self.calc_move_pt()

    def on_exit_move(self):
        self.remove_subbehavior('move')

    def on_enter_kick(self):
        kicker = skills.line_kick.LineKick()
        kicker.use_chipper = True
        kicker.kick_power = 50

        # Pass to point in front of opponent goal
        receive_pt = robocup.Point(0, 3 * constants.Field.Length / 4)

        pass_behavior = tactics.coordinated_pass.CoordinatedPass(
            receive_pt,
            None,
            (kicker, lambda x: True),
            receiver_required=False,
            kicker_required=False,
            prekick_timeout=9)
        self.add_subbehavior(pass_behavior, 'pass')


    def calc_move_pt(self):
        receive_pt = robocup.Point(0, 3 * constants.Field.Length / 4)
        ball = main.ball().pos
        return (ball - receive_pt).normalized() * 0.15 + main.ball().pos
