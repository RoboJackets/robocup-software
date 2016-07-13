import standard_play
import behavior
import skills.move
import skills.pivot_kick
import constants
import robocup
import main
import timeout_behavior
import tactics.coordinated_pass
import evaluation.touchpass_positioning


class OurFreeKick(standard_play.StandardPlay):

    running = False

    def __init__(self, indirect=None):
        super().__init__(continuous=True)

        # If we are indirect we don't want to shoot directly into the goal
        gs = main.game_state()

        if indirect != None:
            self.indirect = indirect
        elif main.ball().pos.y > constants.Field.Length / 2.0:
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
        # This will be reset to something else if indirect on the first iteration
        kicker.target = constants.Field.TheirGoalSegment

        # add two 'centers' that just move to fixed points
        center1 = skills.move.Move(robocup.Point(0, 1.5))
        self.add_subbehavior(center1, 'center1', required=False, priority=4)
        center2 = skills.move.Move(robocup.Point(0, 1.5))
        self.add_subbehavior(center2, 'center2', required=False, priority=3)

        if self.indirect:
            receive_pt, target_point, probability = evaluation.touchpass_positioning.eval_best_receive_point(
                main.ball().pos)
            pass_behavior = timeout_behavior.TimeoutBehavior(tactics.coordinated_pass.CoordinatedPass(receive_pt, None, (kicker, lambda x: True), receiver_required=False, kicker_required=False), 9)
            # We don't need to manage this anymore
            self.add_subbehavior(pass_behavior, 'kicker')

            kicker.target = receive_pt
        else:
            kicker = skills.line_kick.LineKick()
            kicker.target = constants.Field.TheirGoalSegment
            self.add_subbehavior(kicker, 'kicker', required=False, priority=5)

        self.add_transition(
            behavior.Behavior.State.running, behavior.Behavior.State.completed,
            lambda: self.subbehavior_with_name('kicker').is_done_running()
            and self.subbehavior_with_name('kicker').state != timeout_behavior.TimeoutBehavior.State.timeout, 'kicker completes')

    @classmethod
    def score(cls):
        gs = main.game_state()
        return 0 if OurFreeKick.running or (
            gs.is_ready_state() and gs.is_our_free_kick()) else float("inf")

    def execute_running(self):
        if self.indirect \
           and self.subbehavior_with_name('kicker').state == timeout_behavior.TimeoutBehavior.State.timeout:
            self.indirect = False
            self.remove_subbehavior('kicker')
            kicker = skills.line_kick.LineKick()
            kicker.target = constants.Field.TheirGoalSegment
            self.add_subbehavior(kicker, 'kicker', required=False, priority=5)

        if self.indirect:
            passState = self.subbehavior_with_name('kicker').behavior.state
            OurFreeKick.running = passState == tactics.coordinated_pass.CoordinatedPass.State.receiving or \
                                  passState == tactics.coordinated_pass.CoordinatedPass.State.kicking
    def on_enter_running(self):
        OurFreeKick.running = False
    def on_exit_running(self):
        OurFreeKick.running = False

    @classmethod
    def is_restart(cls):
        return True
