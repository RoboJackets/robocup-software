import play
import behavior
import robocup
import tactics.line_up
import tactics.defense
import main


class DefendPenalty(play.Play):

    def __init__(self):
        super().__init__(continuous=True)

        self.add_transition(behavior.Behavior.State.start,
            behavior.Behavior.State.running,
            lambda: True,
            'immediately')

        # lineup
        line = robocup.Segment(robocup.Point(1.5, 1.3), robocup.Point(1.5, 2.5))
        lineup = tactics.line_up.LineUp(line)
        self.add_subbehavior(lineup, 'lineup')

        #self.add_subbehavior(tactics.defense.Defense(), 'defense', required=False,)


    @classmethod
    def score(cls):
        gs = main.game_state()
        return 0 if gs.is_their_penalty() and gs.is_setup_state() else float("inf")

    @classmethod
    def is_restart(cls):
        return True

    @classmethod
    def handles_goalie(cls):
        return True
