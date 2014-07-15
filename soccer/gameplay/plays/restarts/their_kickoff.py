import play
import behavior
import robocup
import tactics.positions.fullback
import tactics.circle_near_ball


class TheirKickoff(play.Play):

    def __init__(self):
        super().__init__(continuous=True)

        self.add_transition(behavior.Behavior.State.start,
            behavior.Behavior.State.running,
            lambda: True,
            'immediately')


        fullback1 = tactics.positions.fullback.Fullback(tactics.positions.fullback.Fullback.Side.left)
        self.add_subbehavior(fullback1, 'fullback1', priority=102)

        fullback2 = tactics.positions.fullback.Fullback(tactics.positions.fullback.Fullback.Side.right)
        self.add_subbehavior(fullback2, 'fullback2', priority=101)

        circle_up = tactics.circle_near_ball.CircleNearBall()
        self.add_subbehavior(circle_up, 'circle_up')


    @classmethod
    def score(cls):
        gs = main.game_state()
        return 0 if gs.is_setup_state() and gs.is_their_kickoff() else float("inf")
