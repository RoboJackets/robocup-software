import standard_play
import behavior
import skills
import tactics
import robocup
import constants
import main
from enum import Enum
import evaluation
from evaluation.passing import eval_pass    


class OurIndirectPivot(standard_play.StandardPlay):

    class State(Enum):
        passing=1
        kicking=2

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
        self.kicker.setup_ball_avoid = constants.Field.CenterRadius - constants.Robot.Radius
        self.kicker.drive_around_dist = constants.Field.CenterRadius - constants.Robot.Radius

        # create a one touch pass behavior with line kick as the skill
        self.pass_bhvr = tactics.coordinated_pass.CoordinatedPass()

        self.add_state(OurIndirectPivot.State.passing,
                       behavior.Behavior.State.running)
        self.add_state(OurIndirectPivot.State.kicking, behavior.Behavior.State.running)

        # add transistions for when the play is done
        self.add_transition(behavior.Behavior.State.start,
                            OurIndirectPivot.State.passing, lambda: True,
                            'immediately')

        self.add_transition(OurIndirectPivot.State.passing,
                            OurIndirectPivot.State.kicking,
                            lambda: self.pass_bhvr.state==behavior.Behavior.State.completed or self.pass_bhvr.state==behavior.Behavior.State.failed, 'Pass Completed')

        self.add_transition(OurIndirectPivot.State.kicking,
                            behavior.Behavior.State.completed,
                            lambda: False, 'Shot Completed')

        # start the actual pass
        self.add_subbehavior(self.pass_bhvr, 'pass')

    @classmethod
    def score(cls):
        gs = main.game_state()

        # enter play when doing a corner kick or stay in it even if we manipulate the ball
        if OurRestartPivot.Running or (gs.is_ready_state() and (gs.is_our_direct() or gs.is_our_indirect()) and main.ball().pos.y < (
                constants.Field.Length - 1.2)):
            OurRestartPivot.Running = True
            return 0
        else:
            return float("inf")

    @classmethod
    def is_restart(cls):
        return True

    def on_enter_passing(self):
        self.add_subbehavior(skills.move.Move(robocup.Point(-main.ball().pos.x,main.ball().pos.y)), 'Threat')

    def execute_passing(self):
        self.pass_bhvr.receive_point = self.pick_pass_spot() 
        self.pass_bhvr.use_chipper = self.evaluate_chip(self.pick_pass_spot())
        

    def on_enter_kicking(self):
        self.remove_all_subbehaviors()
        self.add_subbehavior(self.kicker, 'Shot')
        self.add_subbehavior(skills.move.Move(robocup.Point(-main.ball().pos,robocup.Field.Length-1.2)), 'Rebound')

    def pick_pass_spot(self):
        return robocup.Point(1,constants.Field.Length*3/4)

    def evaluate_chip(self, receive_point):
        bp = main.ball().pos
        ex_robots = []
        kick_p = eval_pass(bp, receive_point, excluded_robots=ex_robots) 
        print("Kick probability is {}".format(kick_p))
        if kick_p < .5:
            ex_robots.extend(evaluation.chipping.chippable_robots())
            chip_p = eval_pass(bp, receive_point, excluded_robots=ex_robots)
            print("Chip probability is {}".format(chip_p))
            if chip_p > kick_p:
                return True
        return False