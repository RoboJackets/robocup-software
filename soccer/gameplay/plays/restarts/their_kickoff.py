import play
import behavior
import robocup
import tactics.positions.defender
import tactics.circle_near_ball
import main


class TheirKickoff(play.Play):

    def __init__(self):
        super().__init__(continuous=True)

        self.add_transition(behavior.Behavior.State.start,
            behavior.Behavior.State.running,
            lambda: True,
            'immediately')


        defender1 = tactics.positions.defender.Defender(tactics.positions.defender.Defender.Side.left)
        self.add_subbehavior(defender1, 'defender1', priority=102)

        defender2 = tactics.positions.defender.Defender(tactics.positions.defender.Defender.Side.right)
        self.add_subbehavior(defender2, 'defender2', priority=101)

        circle_up = tactics.circle_near_ball.CircleNearBall()
        self.add_subbehavior(circle_up, 'circle_up')


    @classmethod
    def score(cls):
        gs = main.game_state()
        return 0 if gs.is_setup_state() and gs.is_their_kickoff() else float("inf")

    @classmethod
    def is_restart(cls):
        return True
