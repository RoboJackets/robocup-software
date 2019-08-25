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

class OurFreeKick(standard_play.StandardPlay):

    Running = False
    BumpKickPower = 0.01
    FullKickPower = 1
    MaxShootingAngle = 80
    # Untested as of now
    MaxChipRange = 3
    MinChipRange = 0.3

    class State(enum.Enum):

        move = 1

        kick = 2

    def __init__(self, indirect=None):
        super().__init__(continuous=True)

        for s in OurFreeKick.State:
            self.add_state(s, behavior.Behavior.State.running)

        # If we are indirect we don't want to shoot directly into the goal
        gs = main.game_state()

        if (main.ball().pos.y > constants.Field.Length / 2):
            self.indirect = gs.is_indirect()
        else:
            self.indirect = False

        self.add_transition(behavior.Behavior.State.start,
                            OurFreeKick.State.move, lambda: True,
                            'immediately')

        self.add_transition(OurFreeKick.State.move, OurFreeKick.State.kick,
                            lambda: (not self.has_subbehavior_with_name(
                                'receiver')) or self.receiver_near_pos(),
                            'kick')

        self.receive_pt, self.receive_value = evaluation.passing_positioning.eval_best_receive_point(
            main.ball().pos, field_weights=(2.0, 10.0, 0.1))
        self.gap = evaluation.shooting.find_gap(
            max_shooting_angle=OurFreeKick.MaxShootingAngle)

        self.add_transition(
            behavior.Behavior.State.running, behavior.Behavior.State.completed,
            lambda: self.subbehavior_with_name('kicker').is_done_running(
            ) and self.subbehavior_with_name('kicker').state != tactics.
            coordinated_pass.CoordinatedPass.State.timeout, 'kicker completes')

    @classmethod
    def score(cls):
        gs = main.game_state()
        return 3 if OurFreeKick.Running or (
            gs.is_ready_state() and gs.is_our_free_kick()) else float("inf")

    def receiver_near_pos(self):
        return len(main.our_robots()) <= 4 or (self.has_subbehavior_with_name('receiver') and (self.subbehavior_with_name('receiver').robot is not None)) and \
            ((self.subbehavior_with_name('receiver').robot.pos - self.pos_up_field).mag() < 0.5)

    def on_enter_move(self):
        self.move_pos = self.calc_move_pos()

        self.pos_up_field = robocup.Point(main.ball().pos.x,
                                          constants.Field.Length * .75)
        if (main.ball().pos.y > constants.Field.Length / 2):
            sign = (main.ball().pos.x) / abs(main.ball().pos.x) * -1
            x = sign * constants.Field.Width * 3 / 8
            y = max(constants.Field.Length * .75,
                    (main.ball().pos.y + constants.Field.Length) * 0.5)
            self.pos_up_field = robocup.Point(x, y)

        if self.indirect and (self.receive_value == 0 and
                              len(main.our_robots()) >= 5):
            self.add_subbehavior(
                skills.move.Move(self.pos_up_field),
                'receiver',
                required=False,
                priority=5)

    def execute_move(self):
        self.move_pos = self.calc_move_pos()

    # def on_exit_move(self):
    # self.remove_all_subbehaviors()

    def on_enter_kick(self):
        OurFreeKick.Running = False
        kicker = skills.line_kick.LineKick()
        kicker.use_chipper = True
        kicker.min_chip_range = OurFreeKick.MinChipRange
        kicker.max_chip_range = OurFreeKick.MaxChipRange

        kicker.target = self.gap

        shooting_line = robocup.Line(main.ball().pos, self.gap)

        # If we are at their goal, shoot full power
        if shooting_line.segment_intersection(constants.Field.TheirGoalSegment) is not None:
            kicker.kick_power = self.FullKickPower
        # If we are aiming in the forward direction and not at one of the "endzones", shoot full power
        elif (shooting_line.line_intersection(constants.Field.FieldBorders[0])
              or
              shooting_line.line_intersection(constants.Field.FieldBorders[2])
              and self.gap.y - main.ball().pos.y > 0):
            kicker.kick_power = self.FullKickPower
        # If we are probably aiming down the field, slowly kick so we dont carpet
        else:
            kicker.kick_power = self.BumpKickPower

        # Try passing if we are doing an indirect kick
        if self.indirect:
            pass
            # Check for valid target pass position
            if self.receive_value != 0 and len(main.our_robots()) >= 5:
                self.remove_all_subbehaviors()
                pass_behavior = tactics.coordinated_pass.CoordinatedPass(
                    self.receive_pt,
                    None, (kicker, lambda x: True),
                    receiver_required=False,
                    kicker_required=False,
                    prekick_timeout=7,
                    use_chipper=True)
                # We don't need to manage this anymore
                self.add_subbehavior(pass_behavior, 'kicker')
            else:
                kicker.target = (self.pos_up_field)
                self.add_subbehavior(kicker, 'kicker', required=False, priority=5)
        else:
            kicker.target = constants.Field.TheirGoalSegment
            self.add_subbehavior(kicker, 'kicker', required=False, priority=5)

    def execute_kick(self):
        if self.indirect \
           and self.subbehavior_with_name('kicker').state == tactics.coordinated_pass.CoordinatedPass.State.timeout:
            self.indirect = False
            self.remove_subbehavior('kicker')
            kicker = skills.line_kick.LineKick()
            kicker.target = constants.Field.TheirGoalSegment
            self.add_subbehavior(kicker, 'kicker', required=False, priority=11)

        if self.indirect:
            passState = self.subbehavior_with_name('kicker').state
            OurFreeKick.Running = passState == tactics.coordinated_pass.CoordinatedPass.State.receiving or \
                                  passState == tactics.coordinated_pass.CoordinatedPass.State.kicking

    def on_exit_kick(self):
        OurFreeKick.Running = False

    @classmethod
    def is_restart(cls):
        return True

    def calc_move_pos(self):
        point = self.gap
        if (self.indirect and self.receive_value != 0):
            point = self.receive_pt
        ball = main.ball().pos
        return (ball - point).normalized() * 0.15 + ball
