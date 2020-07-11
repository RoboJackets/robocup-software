import standard_play
import behavior
import skills
import tactics
import robocup
import constants
import main
import time


class OurCornerKickTouch(standard_play.StandardPlay):

    MinChipRange = 0.3
    MaxChipRange = 3.0
    ChipperPower = 0.5
    TargetSegmentWidth = 1.5
    MaxKickSpeed = 0.5
    MaxKickAccel = 0.5
    Timeout = 10.0

    def __init__(self, indirect=None):
        super().__init__(continuous=False)

        self.start_time = time.time()

        # setup a line kick skill to replace the pivotkick since a pivot would easily cause a double touch
        self.kicker = skills.line_kick.LineKick()
        self.kicker.chip_power = OurCornerKickTouch.ChipperPower  # TODO: base this on the target dist from the bot
        self.kicker.min_chip_range = OurCornerKickTouch.MinChipRange
        self.kicker.max_chip_range = OurCornerKickTouch.MaxChipRange
        self.kicker.max_speed = OurCornerKickTouch.MaxKickSpeed
        self.kicker.max_accel = OurCornerKickTouch.MaxKickAccel

        # larger avoid ball radius for line kick setup so we don't run over the ball backwards
        self.kicker.setup_ball_avoid = constants.Field.CenterRadius - constants.Robot.Radius
        self.kicker.drive_around_dist = constants.Field.CenterRadius - constants.Robot.Radius

        # create a one touch pass behavior with line kick as the kicker skill
        self.pass_bhvr = tactics.one_touch_pass.OneTouchPass(self.kicker)

        # start the actual pass
        self.add_subbehavior(self.pass_bhvr, 'pass')

        # add transistions for when the play is done
        self.add_transition(behavior.Behavior.State.start,
                            behavior.Behavior.State.running, lambda: True,
                            'immediately')

        self.add_transition(behavior.Behavior.State.running,
                            behavior.Behavior.State.completed,
                            self.pass_bhvr.is_done_running, 'passing is done')

        for state in OurCornerKickTouch.State:
            self.add_transition(
                state, behavior.Behavior.State.failed, lambda: (time.time(
                ) - self.start_time > OurCornerKickTouch.Timeout), 'failed')

    @classmethod
    def score(cls):
        gs = main.game_state()
        if len(main.our_robots()) < 5:
            return float("inf")
        if (gs.is_ready_state() and gs.is_our_free_kick() and
                main.ball().pos.y > (constants.Field.Length - 1.2) and
                abs(main.ball().pos.x) > .6):
            return 0
        else:
            return 10000

    @classmethod
    def is_restart(cls):
        return True

    def execute_running(self):
        super().execute_running()
