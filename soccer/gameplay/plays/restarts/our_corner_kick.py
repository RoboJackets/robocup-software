import standard_play
import behavior
import skills
import tactics
import robocup
import constants
import main
import enum


class OurCornerKick(standard_play.StandardPlay):

    MinChipRange = 0.3
    MaxChipRange = 3.0
    ChipperPower = 0.5
    TargetSegmentWidth = 1.5
    MaxKickSpeed = 0.5
    MaxKickAccel = 0.5

    class State(enum.Enum):
        move = 1

        kick = 2

    def __init__(self):
        super().__init__(continuous=True)

        for s in OurCornerKick.State :
            self.add_state(s, behavior.Behavior.State.running)

        self.add_transition(behavior.Behavior.State.start,
                            OurCornerKick.State.move, lambda: True,
                            'immediately')

        self.add_transition(OurCornerKick.State.move,
                            OurCornerKick.State.kick,
                            lambda: self.subbehavior_with_name('move behind').state == behavior.Behavior.State.completed,
                            'kick')

        self.kicker = skills.line_kick.LineKick()
        self.add_transition(OurCornerKick.State.kick,
                            behavior.Behavior.State.completed,
                            self.kicker.is_done_running, 'kicker is done')

    @classmethod
    def score(cls):
        gs = main.game_state()
        if gs.is_ready_state() and gs.is_our_direct() and main.ball().pos.y > (
                constants.Field.Length - 1.0):
            return 1
        else:
            return float("inf")

    @classmethod
    def is_restart(cls):
        return True

    def on_enter_move(self):
        goal_x = constants.Field.GoalWidth * (1 if main.ball().pos.x < 0 else -1)
        self.move_behind_pos = (main.ball().pos - robocup.Segment(
            robocup.Point(goal_x, constants.Field.Length), robocup.Point(
                goal_x,
                constants.Field.Length - OurCornerKick.TargetSegmentWidth)).center()).normalized() * 0.2 + main.ball().pos
        self.add_subbehavior(skills.move.Move(self.move_behind_pos),'move behind', required = True, priority = 5)

    def execute_move(self):
        goal_x = constants.Field.GoalWidth * (1 if main.ball().pos.x < 0 else -1)
        self.move_behind_pos = (main.ball().pos - robocup.Segment(
        robocup.Point(goal_x, constants.Field.Length), robocup.Point(goal_x,
        constants.Field.Length - OurCornerKick.TargetSegmentWidth)).center()).normalized() * 0.2 + main.ball().pos

    def on_exit_move(self):
        self.remove_all_subbehaviors()

    def on_enter_kick(self):
        self.kicker = skills.line_kick.LineKick()
        self.kicker.use_chipper = True
        self.kicker.chip_power = OurCornerKick.ChipperPower  # TODO: base this on the target dist from the bot
        self.kicker.min_chip_range = OurCornerKick.MinChipRange
        self.kicker.max_chip_range = OurCornerKick.MaxChipRange
        self.kicker.max_speed = OurCornerKick.MaxKickSpeed
        self.kicker.max_accel = OurCornerKick.MaxKickAccel
        self.add_subbehavior(self.kicker, 'kicker', required=True, priority=5)

        # larger avoid ball radius for line kick setup so we don't run over the ball backwards
        self.kicker.setup_ball_avoid = constants.Field.CenterRadius - constants.Robot.Radius
        self.kicker.drive_around_dist = constants.Field.CenterRadius - constants.Robot.Radius

        self.center1 = skills.move.Move()
        self.add_subbehavior(self.center1,
                             'center1',
                             required=False,
                             priority=4)

        self.center2 = skills.move.Move()
        self.add_subbehavior(self.center2,
                             'center2',
                             required=False,
                             priority=3)


    def execute_kick(self):
        # setup the kicker target
        goal_x = constants.Field.GoalWidth * (1 if main.ball().pos.x < 0 else
                                              -1)
        target = robocup.Segment(
            robocup.Point(goal_x, constants.Field.Length), robocup.Point(
                goal_x,
                constants.Field.Length - OurCornerKick.TargetSegmentWidth))
        self.kicker.target = target

        # set centers' positions
        center_x_mag = constants.Field.GoalWidth / 2.0 + 0.5
        center_y = constants.Field.Length - OurCornerKick.TargetSegmentWidth / 2.0
        self.center1.target = robocup.Point(center_x_mag, center_y)
        self.center2.target = robocup.Point(-center_x_mag, center_y)
