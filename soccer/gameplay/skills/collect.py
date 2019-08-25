import main
import robocup
import math
import behavior
import single_robot_behavior
import evaluation.ball
import constants


# Moves in to dribble a slow moving ball
class Collect(single_robot_behavior.SingleRobotBehavior):
    # Ball has to be within this distance to be considered captured
    RESTART_MIN_DIST = constants.Robot.Radius + constants.Ball.Radius + 0.05

    # How many of the last X cycles "has_ball()" was true
    PROBABLY_HELD_MAX = 100

    # How many cycles we want held
    PROBABLY_HELD_CUTOFF = 50

    # Really only care about how close robots are to the ball
    # Higher means closer, but possibly less optimal overall with all robot movement
    POSITION_COST_MULTIPLER = 30

    # Restart counter
    # If both robot and ball are stopped near each other, but not in mouth
    # Restart because the path planner doesn't have angle information
    RESTART_TIMEOUT = 100

    STOPPED_VEL = 0.1

    def __init__(self):
        super().__init__(continuous=False)

        self.probably_held_cnt = 0
        self.timeout = 0

        self.add_transition(behavior.Behavior.State.start,
                            behavior.Behavior.State.running, lambda: True,
                            'immediately')

        # Restart the collect path planner
        # Since there is no definition of which direction a robot is facing
        # We cannot figure out if the ball is directly behind us or in it's mouth
        # We can restart the collect and it'll try it again
        # Only restart when the ball is close, both are stopped, and we dont have the ball in the mouth
        self.add_transition(
            behavior.Behavior.State.running,
            behavior.Behavior.State.start, lambda: self.is_bot_ball_stopped(
            ) and not evaluation.ball.robot_has_ball(
                self.robot) and self.probably_held_cnt < Collect.
            PROBABLY_HELD_CUTOFF and self.timeout == 0, 'restart')

        # Complete when we have the ball
        self.add_transition(
            behavior.Behavior.State.running,
            behavior.Behavior.State.completed, lambda: self.robot is not None
            and evaluation.ball.robot_has_ball(self.robot) and self.
            probably_held_cnt > Collect.PROBABLY_HELD_CUTOFF, 'ball collected')

        # Go back if we loose the ball
        self.add_transition(
            behavior.Behavior.State.completed, behavior.Behavior.State.running,
            lambda: self.robot is not None and ((self.robot.pos - main.ball(
            ).pos).mag() > Collect.RESTART_MIN_DIST or self.probably_held_cnt <
                                                Collect.PROBABLY_HELD_CUTOFF),
            'ball lost')

    # Whether both the robot and ball are stopped
    def is_bot_ball_stopped(self):
        return (self.robot.pos - main.ball().pos).mag() < Collect.RESTART_MIN_DIST and \
                self.robot.vel.mag() < Collect.STOPPED_VEL and \
                main.ball().vel.mag() < Collect.STOPPED_VEL

    def on_enter_running(self):
        self.probably_held_cnt = 0

    def execute_running(self):
        if (self.robot is not None):
            self.robot.disable_avoid_ball()
            self.robot.set_dribble_speed(
                constants.Robot.Dribbler.StandardPower)
            self.robot.collect()

            self.update_held_cnt()

            # If the ball and robot are both stopped
            # and it's not in the mouth
            # increase timeout
            if (self.is_bot_ball_stopped() and
                    not evaluation.ball.robot_has_ball(self.robot)):

                self.timeout = min(self.timeout + 1, Collect.RESTART_TIMEOUT)
            else:
                self.timeout = 0

    def execute_completed(self):
        if (self.robot is not None):
            self.robot.disable_avoid_ball()
            self.robot.set_dribble_speed(
                constants.Robot.Dribbler.StandardPower)

            self.update_held_cnt()

    def update_held_cnt(self):
        # If we see the ball, increment up to max
        # if not, drop to 0
        if (evaluation.ball.robot_has_ball(self.robot) or
                not main.ball().valid):  #self.robot.has_ball()):
            self.probably_held_cnt = min(self.probably_held_cnt + 1,
                                         Collect.PROBABLY_HELD_MAX)
        else:
            self.probably_held_cnt = max(self.probably_held_cnt - 1, 0)

    def role_requirements(self):
        reqs = super().role_requirements()

        # try to be near the ball
        if main.ball().valid:
            reqs.destination_shape = main.ball().pos
            reqs.position_cost_multiplier = Collect.POSITION_COST_MULTIPLER

        return reqs
