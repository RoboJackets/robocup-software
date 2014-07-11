import play
import behavior
import robocup
import main
import tactics.line_up
import tactics.penalty


# one robot kicks the ball, the others just line up and wait
class KickPenalty(play.Play):

    def __init__(self):
        super().__init__(continuous=True)

        self.add_transition(behavior.Behavior.State.start,
            behavior.Behavior.State.running,
            lambda: True,
            'immediately')


        kicker = tactics.penalty.Penalty()
        self.add_subbehavior(kicker, 'kicker', required=True, priority=10)

        line = robocup.Segment(robocup.Point(1.5, 1), robocup.Point(1.5, 2.5))
        line_up = tactics.line_up.LineUp(line)


    def score(self):
        gs = main.game_state()
        return 0.0 if gs.is_setup_state() and gs.is_our_penalty() else float("inf")
