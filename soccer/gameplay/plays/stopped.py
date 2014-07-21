import play
import behavior
import tactics.defense
import main


# when we get the Stopped command from the referee,
# we run this play.  See the rules to see what we're allowed to do while the game is stopped
class Stopped(play.Play):
    def __init__(self):
        super().__init__(continuous=True)
        self.add_transition(behavior.Behavior.State.start,
            behavior.Behavior.State.running,
            lambda: True,
            'immediately')

        self.add_subbehavior(tactics.defense.Defense(), 'defense', required=False)

        idle = tactics.circle_near_ball.CircleNearBall()
        self.add_subbehavior(idle, 'circle_up', required=False, priority=1)


    @classmethod
    def score(cls):
        return 0 if main.game_state().is_stopped() else float("inf")


    @classmethod
    def handles_goalie(self):
        return True
