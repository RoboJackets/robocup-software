import play
import behavior
import robocup
import skills
import tactics
import constants
import evaluation
import main


# sends a goal kick towards the goal if it's open
# otherwise, chips/kicks the ball to opposite side of field and
# sends another robot to go intercept
# NOTE: this is a por of the C++ OurGoalKick2, NOT OurGoalKick (it was trashed)
class OurGoalKick(play.Play):

    # tunable params
    MinChipRange = 0.3
    MaxChipRange = 3.0
    KickerPower = 100.0
    ChipperPower = constants.Robot.Chipper.MaxPower


    def __init__(self):
        super().__init__(continuous=True)

        self.add_transition(behavior.Behavior.State.start,
            behavior.Behavior.State.running,
            lambda: True,
            'immediately')


        kicker = skills.line_kick.LineKick()
        # kicker.use_chipper = True
        kicker.kick_power = OurGoalKick.KickerPower
        kicker.chip_power = OurGoalKick.ChipperPower
        self.add_subbehavior(kicker, 'kicker', required=True, priority=6)


        center1 = skills.move.Move()
        self.add_subbehavior(center1, 'center1', required=False, priority=5)
        
        center2 = skills.move.Move()
        self.add_subbehavior(center2, 'center2', required=False, priority=4)


        self.add_subbehavior(tactics.defense.Defense(), 'defense', required=False)



    @classmethod
    def score(cls):
        gs = main.game_state()
        return 0 if (gs.is_ready_state() and gs.is_our_direct() and main.ball().pos.y < 1.0) else float("inf")

    @classmethod
    def is_restart(cls):
        return True

    @classmethod
    def handles_goalie(cls):
        return True


    def execute_running(self):
        kicker = self.subbehavior_with_name('kicker')
        center1 = self.subbehavior_with_name('center1')
        center2 = self.subbehavior_with_name('center2')

        # see if we have a direct shot on their goal
        win_eval = evaluation.window_evaluator.WindowEvaluator()
        win_eval.enable_chip = kicker.robot != None and kicker.robot.has_chipper()
        win_eval.min_chip_range = OurGoalKick.MinChipRange
        win_eval.max_chip_range = OurGoalKick.MaxChipRange
        windows, best = win_eval.eval_pt_to_seg(main.ball().pos, constants.Field.TheirGoalSegment)

        # note: the min length value is tunable
        if best != None and best.segment.length() > 0.3:
            # we DO have a shot on the goal, take it!
            kicker.target = constants.Field.TheirGoalSegment

            # FIXME: make the other robots get out of the shot path

            center1.target = robocup.Point(0.0, 1.5)
            center2.target = robocup.Point(1.0, 1.5)

        else:
            # no open shot, shoot it in-between the two centers
            center_x = constants.Field.Width * 0.15
            center_y = constants.Field.Length * 0.6

            center1.target = robocup.Point(-center_x, center_y)
            center2.target = robocup.Point(center_x, center_y)

            kicker.target = robocup.Segment(center1.target, center2.target)
