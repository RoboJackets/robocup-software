import main
import robocup
import math
import behavior
import single_robot_behavior

class Collect(single_robot_behavior.SingleRobotBehavior):
    # Ball has to be within this distance to be considered captured
    RESTART_MIN_DIST = 0.12

    # Ball has to be below this speed to be considered stopped
    STOP_SPEED = 0.1

    DRIBBLE_SPEED = 254

    # How many of the last X cycles "has_ball()" was true
    PROBABLY_HELD_MAX = 60

    # How many cycles we want held
    PROBABLY_HELD_CUTOFF = 30

    def __init__(self):
        super().__init__(continuous=False)

        self.probably_held_cnt = 0

        self.add_transition(behavior.Behavior.State.start,
                            behavior.Behavior.State.running,
                            lambda: True, 'immediately')

        # Complete when we have the ball and it's stopped
        self.add_transition(behavior.Behavior.State.running,
                            behavior.Behavior.State.completed,
                            lambda: False,#self.robot is not None and
                                    #self.robot.has_ball() and
                                    #self.robot.vel.mag() < Collect.STOP_SPEED, # and
                                    #(self.robot.pos - main.ball().pos).mag() < Collect.RESTART_MIN_DIST and
                                    #self.probably_held_cnt > Collect.PROBABLY_HELD_CUTOFF,
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
            self.robot.set_dribble_speed(Collect.DRIBBLE_SPEED)
            self.robot.disable_avoid_ball()
            self.robot.collect()

            self.update_held_cnt()

    def execute_completed(self):
        if (self.robot is not None):
            self.robot.set_dribble_speed(Collect.DRIBBLE_SPEED)
            self.robot.disable_avoid_ball()

            self.update_held_cnt()

    def update_held_cnt(self):
        # If we see the ball, increment up to max
        # if not, drop to 0
        if (self.robot.has_ball()):
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

        return reqs
