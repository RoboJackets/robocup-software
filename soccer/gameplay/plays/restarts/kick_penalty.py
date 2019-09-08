import standard_play
import behavior
import robocup
import main
import tactics.line_up
import tactics.penalty
import planning_priority


# one robot kicks the ball, the others just line up and wait
class KickPenalty(standard_play.StandardPlay):
    def __init__(self):
        super().__init__(continuous=True)

        self.add_transition(behavior.Behavior.State.start,
                            behavior.Behavior.State.running, lambda: True,
                            'immediately')

        self.add_transition(
            behavior.Behavior.State.running, behavior.Behavior.State.completed,
            lambda: self.kicker.is_done_running(), 'when kicker finishes.')

        self.kicker = tactics.penalty.Penalty()
        self.add_subbehavior(self.kicker,
                             'kicker',
                             required=True,
                             priority=planning_priority.PENALTY_KICKER)

        line = robocup.Segment(robocup.Point(1.5, 1), robocup.Point(1.5, 2.5))
        line_up = tactics.line_up.LineUp(line)

    @classmethod
    def score(cls):
        gs = main.game_state()
        return 0 if gs.is_our_penalty() and not gs.is_penalty_shootout(
        ) else float("inf")

    @classmethod
    def is_restart(cls):
        return True
