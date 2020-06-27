import behavior
import tactics.stopped.circle_near_ball
import tactics.stopped.circle_on_center
import main
import enum
import robocup
import constants
import play

## When we get the Stopped command from the referee, we run this play.
# See the rules to see what we're allowed to do while the game is stopped
class BasicStopped(play.Play):
    class State(enum.Enum):
        normal = 1  # Normal
        center = 2  # Ball is in the center

    def __init__(self):
        super().__init__(continuous=True)

        for state in BasicStopped.State:
            self.add_state(state, behavior.Behavior.State.running)

        self.add_transition(behavior.Behavior.State.start,
                            BasicStopped.State.normal, lambda: True,
                            'immediately')

        self.add_transition(BasicStopped.State.normal,
                            BasicStopped.State.center,
                            lambda: self.is_in_center(),
                            'Switched into center mode')

        self.add_transition(BasicStopped.State.center,
                            BasicStopped.State.normal,
                            lambda: not self.is_in_center(),
                            'Switched into normal mode')

        self.slow_speed = 1
        self.speed = 2.2

    @classmethod
    def score(cls):
        return 0 if main.game_state().is_stopped() else float("inf")

    @classmethod
    def handles_goalie(self):
        return True

    def try_preempt(self) -> bool:
        if(main.game_state().is_stopped()):
            return False
        else:
            self.terminate()
            return True

    ## Allow the stopped play to run during the stopped play.
    # Without this, the stopped play would be killed in the stopped state.
    @classmethod
    def run_during_stopped(cls):
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

    def execute_normal(self):
        for r in main.our_robots():
            r.set_max_speed(self.slow_speed)

    def execute_center(self):
        for r in main.our_robots():
            r.set_max_speed(self.slow_speed)

    def on_exit_center(self):
        for r in main.our_robots():
            r.set_max_speed(self.speed)

    def on_exit_normal(self):
        for r in main.our_robots():
            r.set_max_speed(self.speed)


    def on_enter_center(self):
        self.remove_all_subbehaviors()

        idle = tactics.stopped.circle_on_center.CircleOnCenter()
        self.add_subbehavior(idle, 'circle_up', required=False, priority=1)
