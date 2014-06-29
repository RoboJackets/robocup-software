import play
import behavior
import tactics


# when we get the Stopped command from the referee,
# we run this play.  See the rules to see what we're allowed to do while the game is stopped
class Stopped(play.Play):
    def __init__(self):
        super().__init__(continuous=True)
        self.add_transition(behavior.Behavior.State.start,
            behavior.Behavior.State.running,
            lambda: True,
            'immediately')


    def score(self):
        return 0 if main.game_state().is_stopped() else float("inf")


    def on_enter_running(self):
        left = tactics.positions.fullback.Fullback()
        self.add_subbehavior(left, 'left_fullback', required=False, priority=50)

        right = tactics.positions.fullback.Fullback()
        self.add_subbehavior(right, 'right_fullback', required=False, priority=49)

        idle = tactics.circle_near_ball.CircleNearBall()
        self.add_subbehavior('circle_up', idle, required=False, priority=1)


    def on_exit_running(self):
        for name in ['circle_up', 'right_fullback', 'left_fullback']:
            self.remove_subbehavior(name)
