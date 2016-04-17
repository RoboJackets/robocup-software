import play
import behavior
import skills.move
import skills.pivot_kick
import tactics.defense
import constants
import robocup
import main


class OurFreeKick(play.Play):
    def __init__(self):
        super().__init__(continuous=True)

        self.add_transition(behavior.Behavior.State.start,
                            behavior.Behavior.State.running, lambda: True,
                            'immediately')

        # FIXME: this could also be a PivotKick
        kicker = skills.line_kick.LineKick()
        # kicker.use_chipper = True
        kicker.min_chip_range = 0.3
        kicker.max_chip_range = 3.0
        kicker.target = constants.Field.TheirGoalSegment
        self.add_subbehavior(kicker, 'kicker', required=False, priority=5)

        # add two 'centers' that just move to fixed points
        center1 = skills.move.Move(robocup.Point(0, 1.5))
        self.add_subbehavior(center1, 'center1', required=False, priority=4)
        center2 = skills.move.Move(robocup.Point(0, 1.5))
        self.add_subbehavior(center1, 'center2', required=False, priority=3)

        self.add_subbehavior(tactics.defense.Defense(),
                             'defense',
                             required=False)

        self.add_transition(
            behavior.Behavior.State.running, behavior.Behavior.State.completed,
            lambda: kicker.is_done_running(), 'kicker completes')

    @classmethod
    def score(cls):
        gs = main.game_state()
        return 0 if gs.is_ready_state() and gs.is_our_free_kick() else float(
            "inf")

    @classmethod
    def is_restart(cls):
        return True

    @classmethod
    def handles_goalie(cls):
        return True
