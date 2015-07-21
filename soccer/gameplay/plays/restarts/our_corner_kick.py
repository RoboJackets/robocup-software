import play
import behavior
import skills
import tactics
import robocup
import constants
import main


class OurCornerKick(play.Play):

    MinChipRange = 0.3
    MaxChipRange = 3.0
    ChipperPower = 0.5
    TargetSegmentWidth = 1.5


    def __init__(self):
        super().__init__(continuous=True)

        self.add_transition(behavior.Behavior.State.start,
            behavior.Behavior.State.running,
            lambda: True,
            'immediately')


        self.kicker = skills.line_kick.LineKick()
        # FIXME: settings
        # self.kicker.use_chipper = True
        self.kicker.chip_power = OurCornerKick.ChipperPower # TODO: base this on the target dist from the bot
        self.kicker.min_chip_range = OurCornerKick.MinChipRange
        self.kicker.max_chip_range = OurCornerKick.MaxChipRange
        self.add_subbehavior(self.kicker, 'kicker', required=True, priority=5)

        # larger avoid ball radius for line kick setup so we don't run over the ball backwards
        self.kicker.setup_ball_avoid = constants.Field.CenterRadius - constants.Robot.Radius
        self.kicker.drive_around_dist = constants.Field.CenterRadius - constants.Robot.Radius

        self.center1 = skills.move.Move()
        self.add_subbehavior(self.center1, 'center1', required=False, priority=4)

        self.center2 = skills.move.Move()
        self.add_subbehavior(self.center2, 'center2', required=False, priority=3)

        self.add_subbehavior(tactics.defense.Defense(), 'defense', required=False)

        self.add_transition(behavior.Behavior.State.running,
            behavior.Behavior.State.completed,
            self.kicker.is_done_running,
            'kicker is done')

    @classmethod
    def score(cls):
        gs = main.game_state()

        if gs.is_ready_state() and gs.is_our_direct() and  main.ball().pos.y > ( constants.Field.Length - 1.0 ):
            return 0
        else:
            return float("inf")

    @classmethod
    def is_restart(cls):
        return True

    @classmethod
    def handles_goalie(cls):
        return True


    def execute_running(self):
        # setup the kicker target
        goal_x = constants.Field.GoalWidth * (1 if main.ball().pos.x < 0 else -1)
        target = robocup.Segment(robocup.Point(goal_x, constants.Field.Length),
            robocup.Point(goal_x, constants.Field.Length - OurCornerKick.TargetSegmentWidth))
        self.kicker.target = target

        # set centers' positions
        center_x_mag = constants.Field.GoalWidth / 2.0 + 0.5
        center_y = constants.Field.Length - OurCornerKick.TargetSegmentWidth / 2.0
        self.center1.target = robocup.Point(center_x_mag, center_y)
        self.center2.target = robocup.Point(-center_x_mag, center_y)
