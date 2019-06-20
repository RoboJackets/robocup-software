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

class OurShootoutChip(standard_play.StandardPlay):

    BumpKickPower = 0.01
    FullKickPower = 1
    MaxShootingAngle = 80

    class State(Enum):
        dribbling = 0
        chipping = 1
        shooting = 2

    def __init__(self, indirect=None):
        super().__init__(continuous=True)


        self.add_state(OurShootoutChip.State.dribbling,
                       behavior.Behavior.State.running)
        self.add_state(OurShootoutChip.State.chipping,
                       behavior.Behavior.State.running)
        self.add_state(OurShootoutChip.State.shooting,
                       behavior.Behavior.State.running)

        self.add_transition(behavior.Behavior.State.start,
                            OurShootoutChip.State.dribbling, lambda: True,
                            'immediately')

        self.add_transition(OurShootoutChip.State.chipping,
                            lambda: True,
                            'immediately')

        # grab the ball and charge the goal

        # if they charge us wait for them to get within the free kick hold range and chip it over them
        # if they dont charge us continue driving until we hit the 1 meter range limit and then kick into the goal

        # setup kick
        kicker = skills.line_kick.LineKick()
        kicker.use_chipper = True
        kicker.min_chip_range = OurFreeKick.MinChipRange
        kicker.max_chip_range = OurFreeKick.MaxChipRange

        gap = evaluation.shooting.find_gap(
            max_shooting_angle=OurFreeKick.MaxShootingAngle)

        kicker.target = gap

        shooting_line = robocup.Line(main.ball().pos, gap)

    @classmethod
    def score(cls):
        gs = main.game_state()
        return 0 if gs.is_penalty_shootout and gs.is_our_penalty() else float("inf")

    def execute_running(self):
            self.remove_subbehavior('kicker')
            kicker = skills.line_kick.LineKick()
            kicker.target = constants.Field.TheirGoalSegment
            self.add_subbehavior(kicker, 'kicker', required=False, priority=5)

    def on_enter_running(self):
        OurFreeKick.Running = False

    def on_exit_running(self):
        OurFreeKick.Running = False

    @classmethod
    def is_restart(cls):
        return True
