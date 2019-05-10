import main
import robocup
import math
import behavior
import single_robot_behavior
import evaluation.ball

class Collect(single_robot_behavior.SingleRobotBehavior):
    # Ball has to be within this distance to be considered captured
    RESTART_MIN_DIST = 0.12

    # Ball has to be below this speed to be considered stopped
    STOP_SPEED = 0.05

    DRIBBLE_SPEED = 70

    # How many of the last X cycles "has_ball()" was true
    PROBABLY_HELD_MAX = 100

    # How many cycles we want held
    PROBABLY_HELD_CUTOFF = 50

    # Really only care about how close robots are to the ball
    # Higher means closer, but possibly less optimal overall with all robot movement
    POSITION_COST_MULTIPLER = 30

    def __init__(self):
        super().__init__(continuous=False)

        self.probably_held_cnt = 0

        self.add_transition(behavior.Behavior.State.start,
                            behavior.Behavior.State.running,
                            lambda: True, 'immediately')

        # Complete when we have the ball and it's stopped
        self.add_transition(behavior.Behavior.State.running,
                            behavior.Behavior.State.completed,
                            lambda: self.robot is not None and
                                    self.robot.has_ball() and
                                    #self.robot.vel.mag() < Collect.STOP_SPEED and # and
                                    #(self.robot.pos - main.ball().pos).mag() < Collect.RESTART_MIN_DIST and
                                    self.probably_held_cnt > Collect.PROBABLY_HELD_CUTOFF,
                            'ball collected')

        self.add_transition(behavior.Behavior.State.completed,
                            behavior.Behavior.State.running,
                            lambda: False and self.robot is not None and
                                    ((self.robot.pos - main.ball().pos).mag() > Collect.RESTART_MIN_DIST or
                                     self.probably_held_cnt < Collect.PROBABLY_HELD_CUTOFF),
                            'ball lost')

    def on_enter_running(self):
        self.probably_held_cnt = 0

    def execute_running(self):
        if (self.robot is not None):
            self.robot.disable_avoid_ball()
            self.robot.set_dribble_speed(Collect.DRIBBLE_SPEED)
            self.robot.collect()

            self.update_held_cnt()

    def execute_completed(self):
        if (self.robot is not None):
            self.robot.disable_avoid_ball()
            self.robot.set_dribble_speed(Collect.DRIBBLE_SPEED)

            self.update_held_cnt()

    def update_held_cnt(self):
        # If we see the ball, increment up to max
        # if not, drop to 0
        if (evaluation.ball.robot_has_ball(self.robot) or not main.ball().valid): #self.robot.has_ball()):
        #if (self.robot is not None and self.robot.vel.mag() < Collect.STOP_SPEED):
            self.probably_held_cnt = min(self.probably_held_cnt + 1,
                                         Collect.PROBABLY_HELD_MAX)
        else:
            self.probably_held_cnt = max(self.probably_held_cnt - 1, 0)

    def role_requirements(self):
        reqs = super().role_requirements()
        #reqs.require_kicking = True
        
        # try to be near the ball
        if main.ball().valid:
            reqs.destination_shape = main.ball().pos
            reqs.position_cost_multiplier = Collect.POSITION_COST_MULTIPLER

        return reqs
