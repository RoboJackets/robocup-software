import standard_play
import behavior
import tactics.defense
import tactics.stopped.circle_near_ball
import tactics.stopped.circle_on_center
import main
import enum
import robocup
import constants


## When we get the Stopped command from the referee, we run this play.
# See the rules to see what we're allowed to do while the game is stopped
class Stopped(standard_play.StandardPlay):
    class State(enum.Enum):
        normal = 1  # Normal
        center = 2  # Ball is in the center

    def __init__(self):
        super().__init__(continuous=True)

        for state in Stopped.State:
            self.add_state(state, behavior.Behavior.State.running)

        self.add_transition(behavior.Behavior.State.start,
                            Stopped.State.normal, lambda: True, 'immediately')

        self.add_transition(Stopped.State.normal, Stopped.State.center,
                            lambda: self.is_in_center(),
                            'Switched into center mode')

        self.add_transition(Stopped.State.center, Stopped.State.normal,
                            lambda: not self.is_in_center(),
                            'Switched into normal mode')

    @classmethod
    def score(cls):
        return 0 if main.game_state().is_stopped() else float("inf")

    @classmethod
    def handles_goalie(self):
        return True

    def is_in_center(self):
        if main.ball().valid:
            return robocup.Circle(
                constants.Field.CenterPoint,
                constants.Field.CenterRadius).contains_point(main.ball().pos)
        return False

    def on_enter_normal(self):
        self.remove_all_subbehaviors()
        
        idle = tactics.stopped.circle_near_ball.CircleNearBall()
        self.add_subbehavior(idle, 'circle_up', required=False, priority=1)

    def on_enter_center(self):
        self.remove_all_subbehaviors()
        
        idle = tactics.stopped.circle_on_center.CircleOnCenter()
        self.add_subbehavior(idle, 'circle_up', required=False, priority=1)
