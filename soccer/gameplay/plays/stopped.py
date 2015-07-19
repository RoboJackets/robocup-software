import play
import behavior
import tactics.defense
import tactics.stopped_plays.circle_near_ball
import tactics.stopped_plays.circle_on_center
import main
import enum
import robocup
import constants


# when we get the Stopped command from the referee,
# we run this play.  See the rules to see what we're allowed to do while the game is stopped
class Stopped(play.Play):

    class State(enum.Enum):
        normal = 1   # Normal
        center = 2   # Ball is in the center

    def __init__(self):
        super().__init__(continuous=True)

        for state in Stopped.State:
            self.add_state(state, behavior.Behavior.State.running)

        self.add_transition(behavior.Behavior.State.start,
            Stopped.State.normal,
            lambda: True,
            'immediately')

        self.add_transition(Stopped.State.normal,
            Stopped.State.center,
            lambda: self.isInCenter(),
            'Switched into center mode')

        self.add_transition(Stopped.State.center,
            Stopped.State.normal,
            lambda: not self.isInCenter(),
            'Switched into normal mode')


    @classmethod
    def score(cls):
        return 0 if main.game_state().is_stopped() else float("inf")


    @classmethod
    def handles_goalie(self):
        return True

    def isInCenter(self):
        if main.ball().valid:
            return robocup.Circle(constants.Field.CenterPoint, constants.Field.CenterRadius).contains_point(main.ball().pos)
        return False

    def on_enter_normal(self):
        self.remove_all_subbehaviors()
        self.add_subbehavior(tactics.defense.Defense(), 'defense', required=False)
        idle = tactics.stopped_plays.circle_near_ball.CircleNearBall()
        self.add_subbehavior(idle, 'circle_up', required=False, priority=1)

    def on_enter_center(self):
        self.remove_all_subbehaviors()
        self.add_subbehavior(tactics.defense.Defense(), 'defense', required=False)
        idle = tactics.stopped_plays.circle_on_center.CircleOnCenter()
        self.add_subbehavior(idle, 'circle_up', required=False, priority=1)

