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
    ChipperPower = constants.Robot.Chipper.MaxPower / 2.0
    TargetSegmentWidth = 1.5


    def __init__(self):
        super().__init__(continuous=True)

        self.add_transition(behavior.Behavior.State.start,
            behavior.Behavior.State.running,
            lambda: True,
            'immediately')


        kicker = skills.line_kick.LineKick()
        # FIXME: settings
        kicker.use_chipper = True
        kicker.chip_power = OurCornerKick.ChipperPower # TODO: base this on the target dist from the bot
        kicker.min_chip_range = OurCornerKick.MinChipRange
        kicker.max_chip_range = OurCornerKick.MaxChipRange
        self.add_subbehavior(kicker, 'kicker', required=True, priority=5)

        center1 = skills.move.Move()
        self.add_subbehavior(center1, 'center1', required=False, priority=4)

        center2 = skills.move.Move()
        self.add_subbehavior(center2, 'center2', required=False, priority=3)


        fullback1 = tactics.positions.fullback.Fullback(side=tactics.positions.fullback.Fullback.Side.left)
        self.add_subbehavior(fullback1, 'fullback1', required=False, priority=2)

        fullback2 = tactics.positions.fullback.Fullback(side=tactics.positions.fullback.Fullback.Side.left)
        self.add_subbehavior(fullback2, 'fullback2', required=False, priority=1)



    # note: the old C++ version of this play required a chipper
    @classmethod
    def score(cls):
        gs = main.game_state()
        chipper_available = any([bot.has_chipper() and bot.shell_id() != main.root_play().goalie_id() for bot in main.our_robots()])

        raise NotImplementedError("old one required a chipper, do we want that here too?")

        return 0 if (gs.is_setup_state() and gs.is_our_direct()
            and main.ball().pos.y > constants.Field.Length - 1.0
            and chipper_available) else float("inf")


    def execute_running(self):
        # setup the kicker target
        goal_x = constants.Field.GoalWidth * (1 if main.ball().pos < 0 else -1)
        target = robocup.Segment(robocup.Point(goal_x, constants.Field.Length),
            robocup.Point(goal_x, constants.Field.Length - OurCornerKick.TargetSegmentWidth))
        kicker.target = target

        # set centers' positions
        center_x_mag = constants.Field.GoalWidth / 2.0 + 0.5
        center_y = constants.Field.Length - OurCornerKick.TargetSegmentWidth / 2.0
        center1.target = robocup.Point(center_x_mag, center_y)
        center2.target = robocup.Point(-center_x_mag, center_y)
