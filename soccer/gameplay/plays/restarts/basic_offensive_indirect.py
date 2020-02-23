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
import situational_play_selection

# A basic play for the Offensive Indect kick
# Chooses to pass to a close or far robot
# If it chooses the far bot, it will chip
# If the ball is in the thrid quarter of the field,
# The robots will move up
# If the ball is in the fourth quearter of the field,
# The robots will move to points in a horizontal line with the ball
#


class BasicOffensiveIndirect(standard_play.StandardPlay):

    _situationList = [
        situational_play_selection.SituationalPlaySelector.Situation.OFFENSIVE_KICK
    ] # yapf: disable

    class State(enum.Enum):
        move = 1  # Move recievers to proper postions
        kick = 2  # Kick the ball to one of the recievers
        backup_kick = 3 # Kick to the other receiver if the first one is blocked

    def __init__(self, indirect=None):
        super().__init__(continuous=True)

        for s in BasicOffensiveIndirect.State:
            self.add_state(s, behavior.Behavior.State.running)

        self.points = []  # The list that holds the receive points

        self.chip = False  # Determines wether to chip or not

        self.receive_pt = None

        self.add_transition(behavior.Behavior.State.start,
                            BasicOffensiveIndirect.State.move, lambda: True,
                            'immediately')

        self.add_transition(
            BasicOffensiveIndirect.State.move,
            BasicOffensiveIndirect.State.kick,
            lambda: self.subbehavior_with_name(
                'move to point 2').state == behavior.Behavior.State.completed,
            'kick')  # Once the receivers are in position

        self.add_transition(
            BasicOffensiveIndirect.State.kick,
            behavior.Behavior.State.completed,
            lambda: self.subbehavior_with_name('pass').state == behavior.
            Behavior.State.completed and self.subbehavior_with_name('pass').
            state != tactics.coordinated_pass.CoordinatedPass.State.timeout,
            # Keep trying pass until timeout
            'pass completes')  # The pass is complete

        self.add_transition(
            BasicOffensiveIndirect.State.kick,
            BasicOffensiveIndirect.State.backup_kick,
            lambda: self.subbehavior_with_name('pass').state == behavior.Behavior.State.failed, 'first pass failed')

        self.add_transition(
            BasicOffensiveIndirect.State.backup_kick,
            behavior.Behavior.State.completed,
            lambda: self.subbehavior_with_name('pass2').state == behavior.
            Behavior.State.completed and self.subbehavior_with_name('pass2').
            state != tactics.coordinated_pass.CoordinatedPass.State.timeout,
            # Keep trying pass until timeout
            'backup pass completes')  # The pass is complete

    @classmethod
    def score(cls):
        gs = main.game_state()
        return 0 if behavior.Behavior.State.running or (
            gs.is_ready_state()
            and gs.is_our_indirect_kick()) else float("inf")

    @classmethod
    def is_restart(cls):
        return True

    def calc_pass_point(
        self):  # Determines the position of the recievers and where to pass
        ball = main.ball().pos  # position of ball
        self.num = random.randint(
            0, 1)  # a random int to choose which receiver to pass to
        if (self.num == 1):
            self.chip = True  # If the reciever is the second one, it will be far away and require a chip
        else:
            self.chip = False
        if (ball.x < 0):
            sign = 1  # If the ball is on the left side of the field the receivers need to be on the right
        else:
            sign = -1
        if -constants.Field.Width / 4 < ball.x and ball.x < constants.Field.Width / 4:  # If the ball is near the center then the close receiver needs to be on the same side as the ball
            ball_mid = -1
        else:
            ball_mid = 1

        offense_point_1 = robocup.Point(
            ball.x + (ball_mid * sign * constants.Field.Width / 4),
            ball.y)  # The close receiver point
        offense_point_2 = robocup.Point(ball.x +
                                        (sign * constants.Field.Width * 0.9),
                                        ball.y)  # The further receive point
        if not (-constants.Field.Width / 2 < offense_point_2.x
                and offense_point_2.x < constants.Field.Width / 2):
            offense_point_2.x = sign * constants.Field.Width * 0.45
            # If the far point will be outside of the field, make it not be
        if (ball.y < constants.Field.Length * 0.75):
            offense_point_2.y += constants.Field.Length * 0.07
            offense_point_1.y += constants.Field.Length * 0.1
            # If the ball is near the midfield have the recievers move further up
        self.points = [offense_point_1, offense_point_2
                       ]  #insert the recieve points into the list
        pass_point = self.points[
            self.num]  # Use the randomly generated int to choose the receiver that will be passed to
        return pass_point

    def on_enter_move(self):  # Move receivers to calculated points
        count = 0
        self.receive_pt = self.calc_pass_point()
        for i in self.points:
            count += 1
            self.add_subbehavior(skills.move.Move(i),
                                 'move to point ' + str(count))

    def on_enter_kick(self):
        self.remove_subbehavior('move to point 1')
        self.remove_subbehavior('move to point 2')
        if self.chip:
            keep_at = 0
        else:
            keep_at = 1
        self.add_subbehavior(skills.move.Move(self.points[keep_at]),
                             'stay at pos')
        kicker = skills.line_kick.LineKick()
        kicker.use_chipper = True
        kicker.kick_power = 50
        # Pass to chosen receiver
        pass_behavior = tactics.coordinated_pass.CoordinatedPass(
            self.receive_pt,
            None, (kicker, lambda x: True),
            receiver_required=False,
            kicker_required=False,
            prekick_timeout=100,
            use_chipper=self.chip)
        self.add_subbehavior(pass_behavior, 'pass')

    def on_exit_kick(self):
        self.remove_subbehavior('pass')
        self.remove_subbehavior('stay at pos')

    def on_enter_backup_kick(self):
        self.remove_all_subbehaviors()
        if self.chip:
            keep_at = 0
        else:
            keep_at = 1
        self.add_subbehavior(skills.move.Move(self.points[keep_at]),
                             'stay at pos 2')
        kicker = skills.line_kick.LineKick()
        kicker.use_chipper = True
        kicker.kick_power = 50
        # Pass to chosen receiver
        pass_behavior = tactics.coordinated_pass.CoordinatedPass(
            self.points[1 - self.num],
            None, (kicker, lambda x: True),
            receiver_required=True,
            kicker_required=False,
            prekick_timeout=100,
            use_chipper=self.chip)
        self.add_subbehavior(pass_behavior, 'pass 2')

    def on_exit_backup_kick(self):
        self.remove_all_subbehaviors()
