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


class OurFreeKick(standard_play.StandardPlay):

    Running = False
    bump_power = 0.05
    full_power = 1

    def __init__(self, indirect=None):
        super().__init__(continuous=True)

        # If we are indirect we don't want to shoot directly into the goal
        gs = main.game_state()

        if (main.ball().pos.y > constants.Field.Length / 2):
            self.indirect = gs.is_indirect()
        else:
            self.indirect = False

        self.add_transition(behavior.Behavior.State.start,
                            behavior.Behavior.State.running, lambda: True,
                            'immediately')

        # FIXME: this could also be a PivotKick
        kicker = skills.line_kick.LineKick()
        # kicker.use_chipper = True
        kicker.min_chip_range = 0.3
        kicker.max_chip_range = 3.0

        target = constants.Field.TheirGoalSegment.center() 

        if self.indirect:
            if main.ball().pos.y > constants.Field.Length / 2:
                target = robocup.Point(constants.Field.Width / 2, constants.Field.Length)
            elif main.ball().pos.y < constants.Field.Length / 2:
                target = robocup.Point(-constants.Field.Width / 2, constants.Field.Length)           

        gap = evaluation.shooting.find_gap(target_pos=target, max_shooting_angle=80)

        kicker.target = gap

        shooting_line = robocup.Line(main.ball().pos, gap)
        
        left_border = robocup.Segment(robocup.Point(-constants.Field.Width / 2.0, 0),
                             robocup.Point(-constants.Field.Width / 2.0, constants.Field.Length))
        right_border = robocup.Segment(robocup.Point(constants.Field.Width / 2.0, 0),
                             robocup.Point(constants.Field.Width / 2.0, constants.Field.Length))
        
        if shooting_line.segment_intersection(constants.Field.TheirGoalSegment) is not None:
            kicker.kick_power = self.full_power
        elif (shooting_line.segment_intersection(left_border) is not None or shooting_line.segment_intersection(right_border) is not None) and main.ball().pos.y < constants.Field.Length / 4:
            kicker.kick_power = self.full_power
        else:
            kicker.kick_power = self.bump_power 

        if self.indirect:
            receive_pt = evaluation.passing_positioning.eval_best_receive_point(main.ball().pos)
            pass_behavior = tactics.coordinated_pass.CoordinatedPass(
                receive_pt,
                None,
                (kicker, lambda x: True),
                receiver_required=False,
                kicker_required=False,
                prekick_timeout=9)
            # We don't need to manage this anymore
            self.add_subbehavior(pass_behavior, 'kicker')
        else:
            self.add_subbehavior(kicker, 'kicker', required=False, priority=5)

        self.add_transition(
            behavior.Behavior.State.running, behavior.Behavior.State.completed,
            lambda: self.subbehavior_with_name('kicker').is_done_running() and self.subbehavior_with_name('kicker').state != tactics.coordinated_pass.CoordinatedPass.State.timeout,
            'kicker completes')

    @classmethod
    def score(cls):
        gs = main.game_state()
        return 0 if OurFreeKick.Running or (
            gs.is_ready_state() and gs.is_our_free_kick()) else float("inf")

    def execute_running(self):
        target = constants.Field.TheirGoalSegment.center() 
        if main.ball().pos.y > constants.Field.Length / 2:
            target = robocup.Point(constants.Field.Width / 2, constants.Field.Length)
        elif main.ball().pos.y < constants.Field.Length / 2:
            target = robocup.Point(-constants.Field.Width / 2, constants.Field.Length)           

        evaluation.shooting.find_gap(target_pos=target, max_shooting_angle=70)

        if self.indirect \
           and self.subbehavior_with_name('kicker').state == tactics.coordinated_pass.CoordinatedPass.State.timeout:
            self.indirect = False
            self.remove_subbehavior('kicker')
            kicker = skills.line_kick.LineKick()
            kicker.target = constants.Field.TheirGoalSegment
            self.add_subbehavior(kicker, 'kicker', required=False, priority=5)

        if self.indirect:
            passState = self.subbehavior_with_name('kicker').state
            OurFreeKick.Running = passState == tactics.coordinated_pass.CoordinatedPass.State.receiving or \
                                  passState == tactics.coordinated_pass.CoordinatedPass.State.kicking

    def on_enter_running(self):
        OurFreeKick.Running = False

    def on_exit_running(self):
        OurFreeKick.Running = False

    @classmethod
    def is_restart(cls):
        return True
