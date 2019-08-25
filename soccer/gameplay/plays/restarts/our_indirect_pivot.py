import standard_play
import behavior
import skills
import tactics
import robocup
import constants
import main
import enum
import evaluation.passing


class OurIndirectPivot(standard_play.StandardPlay):
    class State(enum.Enum):
        passing = 1
        kicking = 2

    MinChipRange = 0.3
    MaxChipRange = 3.0
    ChipperPower = 1
    TargetSegmentWidth = 1.5
    MaxKickSpeed = 1
    MaxKickAccel = 1
    Running = False

    def __init__(self, indirect=None):
        super().__init__(continuous=True)

        # setup a line kick skill to replace the pivotkick since a pivot would easily cause a double touch
        self.kicker = skills.pivot_kick.PivotKick()
        self.kicker.chip_power = OurIndirectPivot.ChipperPower  # TODO: base this on the target dist from the bot
        self.kicker.min_chip_range = OurIndirectPivot.MinChipRange
        self.kicker.max_chip_range = OurIndirectPivot.MaxChipRange
        self.kicker.max_speed = OurIndirectPivot.MaxKickSpeed
        self.kicker.max_accel = OurIndirectPivot.MaxKickAccel

        # larger avoid ball radius for line kick setup so we don't run over the ball backwards
        #self.kicker.setup_ball_avoid = constants.Field.CenterRadius - constants.Robot.Radius
        #self.kicker.drive_around_dist = constants.Field.CenterRadius - constants.Robot.Radius

        # create a one touch pass behavior with line kick as the skill
        self.pass_bhvr = tactics.coordinated_pass.CoordinatedPass()

        self.add_state(OurIndirectPivot.State.passing,
                       behavior.Behavior.State.running)
        self.add_state(OurIndirectPivot.State.kicking,
                       behavior.Behavior.State.running)

        # add transistions for when the play is done
        self.add_transition(behavior.Behavior.State.start,
                            OurIndirectPivot.State.passing, lambda: True,
                            'immediately')

        self.add_transition(OurIndirectPivot.State.passing,
                            OurIndirectPivot.State.kicking, lambda: self.
                            pass_bhvr.is_done_running(), 'Pass Completed')

        self.add_transition(OurIndirectPivot.State.kicking,
                            behavior.Behavior.State.completed, lambda: False,
                            'Shot Completed')

    @classmethod
    def score(cls):
        gs = main.game_state()

        # enter play when doing a corner kick or stay in it even if we manipulate the ball
        if OurIndirectPivot.Running or (
                gs.is_ready_state() and
            (gs.is_our_direct() or gs.is_our_indirect()) and
                main.ball().pos.y < (constants.Field.Length - 1.2)):
            OurIndirectPivot.Running = True
            return 0
        else:
            return float("inf")

    def on_enter_passing(self):
        # start the actual pass
        self.add_subbehavior(self.pass_bhvr, 'pass')
        self.pass_bhvr.receive_point = self.pick_pass_spot()
        self.pass_bhvr.use_chipper = self.evaluate_chip(self.pick_pass_spot())

        if abs(main.ball().pos.x) > 2:
            threat_point = robocup.Point(-2 * main.ball().pos.x /
                                         main.ball().pos.x, main.ball().pos.y)
        elif abs(main.ball().pos.x) < .25:
            threat_point = robocup.Point(-(.5 + main.ball().pos.x),
                                         main.ball().pos.y)
        else:
            threat_point = robocup.Point(-main.ball().pos.x, main.ball().pos.y)
        self.add_subbehavior(
            skills.move.Move(threat_point),
            'Threat',
            required=False,
            priority=1)

    def execute_passing(self):
        if self.pass_bhvr.state == tactics.coordinated_pass.CoordinatedPass.State.preparing:
            self.pass_bhvr.receive_point = self.pick_pass_spot()
            self.pass_bhvr.use_chipper = self.evaluate_chip(
                self.pick_pass_spot())
        print(self.pass_bhvr.state)

    def on_enter_kicking(self):
        self.remove_all_subbehaviors()
        self.add_subbehavior(self.kicker, 'Shot')

        if abs(main.ball().pos.x) > 2:
            rebound_point = robocup.Point(-2 * main.ball().pos.x /
                                          main.ball().pos.x, main.ball().pos.y)
        elif abs(main.ball().pos.x) < .25:
            rebound_point = robocup.Point(-(.5 + main.ball().pos.x),
                                          main.ball().pos.y)
        else:
            rebound_point = robocup.Point(-main.ball().pos.x,
                                          main.ball().pos.y)

        self.add_subbehavior(
            skills.move.Move(rebound_point),
            'Rebound',
            required=False,
            priority=5)

    def execute_kicking(self):
        print(self.subbehavior_with_name('Shot').state)
        if self.subbehavior_with_name('Shot').state in [
                behavior.Behavior.State.completed,
                behavior.Behavior.State.failed
        ]:
            OurIndirectPivot.Running = False

    def on_exit_kicking(self):
        self.remove_all_subbehaviors()

    def pick_pass_spot(self):
        return robocup.Point(1, constants.Field.Length * 3 / 4)

    def evaluate_chip(self, receive_point):
        bp = main.ball().pos
        ex_robots = []
        kick_p = evaluation.passing.eval_pass(
            bp, receive_point, excluded_robots=ex_robots)

        if kick_p < .5:
            ex_robots.extend(evaluation.chipping.chippable_robots())
            chip_p = evaluation.passing.eval_pass(
                bp, receive_point, excluded_robots=ex_robots)

            if chip_p > kick_p:
                return True
        return False

    @classmethod
    def is_restart(cls):
        return True
