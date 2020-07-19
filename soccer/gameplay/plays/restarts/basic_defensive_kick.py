import main
import robocup
import behavior
import constants

import standard_play
import skills.line_kick
import tactics.coordinated_pass
import enum
import skills.move
import random
from situations import Situation

## A basic play for the Defensive restart kick
# Will have two robots move up onto offensive half of field
# will chip to the one directly in front of the kicking robot


class BasicDefensiveKick(standard_play.StandardPlay):

    _situationList = [
        Situation.DEFENSIVE_KICK
    ] # yapf: disable

    class State(enum.Enum):
        move = 1  # Move receivers to proper postions
        kick = 2  # Kick the ball to one of the receivers

    def __init__(self, indirect=None):
        super().__init__(continuous=True)

        for s in BasicDefensiveKick.State:
            self.add_state(s, behavior.Behavior.State.running)

        self.points = []  # The list that holds the receive points

        self.chip = True  # Determines wether to chip or not

        self.add_transition(behavior.Behavior.State.start,
                            BasicDefensiveKick.State.move, lambda: True,
                            'immediately')

        self.add_transition(
            BasicDefensiveKick.State.move, BasicDefensiveKick.State.kick,
            lambda: self.subbehavior_with_name(
                'move to point 2').state == behavior.Behavior.State.completed,
            'kick')  # Once the receivers are in position

        self.add_transition(
            BasicDefensiveKick.State.kick,
            behavior.Behavior.State.completed,
            lambda: self.subbehavior_with_name('pass').state == behavior.
            Behavior.State.completed and self.subbehavior_with_name('pass').
            state != tactics.coordinated_pass.CoordinatedPass.State.timeout,
            # Keep trying pass until timeout
            'pass completes')  # The pass is complete

    @classmethod
    def score(cls):
        gs = main.game_state()
        return 0 if behavior.Behavior.State.running or (
            gs.is_ready_state()
            and gs.is_our_indirect_kick()) else float("inf")

    @classmethod
    def is_restart(cls):
        return True

    def on_enter_move(self):  # Move receivers to calculated points
        ball = main.ball().pos
        self.movepoint1 = robocup.Point(
            ball.x, constants.Field.Length / 2 + ball.y / 2)
        self.movepoint2 = robocup.Point(
            -ball.x, constants.Field.Length / 2 + ball.y / 2)
        if abs(ball.x) < constants.Field.Width / 6:
            self.movepoint2.x = ball.x + constants.Field.Width / 4
        self.points = [self.movepoint1, self.movepoint2]
        count = 0
        for i in self.points:
            count += 1
            self.add_subbehavior(skills.move.Move(i),
                                 'move to point ' + str(count))

    def on_enter_kick(self):
        self.remove_subbehavior('move to point 1')
        self.remove_subbehavior('move to point 2')
        kicker = skills.line_kick.LineKick()
        kicker.use_chipper = True
        kicker.kick_power = 50
        # Pass to chosen receiver
        pass_behavior = tactics.coordinated_pass.CoordinatedPass(
            self.points[0],
            None, (kicker, lambda x: True),
            receiver_required=True,
            kicker_required=False,
            prekick_timeout=100,
            use_chipper=self.chip)
        self.add_subbehavior(pass_behavior, 'pass')

    def on_exit_kick(self):
        self.remove_subbehavior('pass')
