import standard_play
import behavior
import skills.move
import skills.pivot_kick
import constants
import robocup
import main
import evaluation.touchpass_positioning


class OurFreeKick(standard_play.StandardPlay):

    tpass = evaluation.touchpass_positioning

    def __init__(self):
        super().__init__(continuous=True)

        # If we are indirect we don't want to shoot directly into the goal
        gs = main.game_state()
        self.indirect = gs.is_indirect()

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
        self.add_subbehavior(kicker, 'kicker', required=False, priority=5)

        # add two 'centers' that just move to fixed points
        center1 = skills.move.Move(robocup.Point(0, 1.5))
        self.add_subbehavior(center1, 'center1', required=False, priority=4)
        center2 = skills.move.Move(robocup.Point(0, 1.5))
        self.add_subbehavior(center2, 'center2', required=False, priority=3)

        self.add_transition(
            behavior.Behavior.State.running, behavior.Behavior.State.completed,
            lambda: kicker.is_done_running(), 'kicker completes')

        if self.indirect:
            receive_pt, target_point, probability = OurFreeKick.tpass.eval_best_receive_point(main.ball().pos)
            recv = skills.move.Move(receive_pt)
            self.add_subbehavior(recv, 'receiver', required=False, priority=5)
            kicker.target = receive_pt

    @classmethod
    def score(cls):
        gs = main.game_state()
        return 0 if gs.is_ready_state() and gs.is_our_free_kick() else float(
            "inf")

    @classmethod
    def is_restart(cls):
        return True
