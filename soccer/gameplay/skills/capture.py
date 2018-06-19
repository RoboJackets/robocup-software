import single_robot_behavior
import behavior
from enum import Enum
import main
import evaluation
import constants
import role_assignment
import robocup
import planning_priority
import time
import skills.move
import math


class Capture(single_robot_behavior.SingleRobotBehavior):

    # tunable config values
    ## Speed in m at which a capture will be handled by coarse and fine approach instead of intercept
    InterceptVelocityThresh = 0.1

    DampenMult = 0.0

    # Coarse Approach Tunables
    CoarseApproachErrorThresh = 0.8
    CoarseApproachVelocity = 0.1
    CoarseApproachDist = 0.05
    CoarseApproachAvoidBall = 0.5

    ## Time in which to wait in delay state to confirm the robot has the ball
    DelayTime = 0.4

    # Default dribbler speed, can be overriden by self.dribbler_power
    ## Sets dribbler speed during intercept and fine approach
    DribbleSpeed = 128
    FineApproachSpeed = 0.3

    InFrontOfBallCosOfAngleThreshold = 0.95

    class State(Enum):
        intercept = 0
        coarse_approach = 1
        fine_approach = 2
        delay = 3

    ## Capture Constructor
    # faceBall - If false, any turning functions are turned off,
    # useful for using capture to reflect/bounce moving balls.
    def __init__(self, faceBall=True):
        super().__init__(continuous=False)

        self.dribbler_power = Capture.DribbleSpeed

        # Declare all states to have a running state
        for state in Capture.State:
            self.add_state(state, behavior.Behavior.State.running)

        # State Transistions
        self.add_transition(
            behavior.Behavior.State.start, Capture.State.intercept,
            lambda: True, 'immediately')

        self.add_transition(
            Capture.State.intercept, Capture.State.coarse_approach,
            lambda: main.ball().vel.mag() < Capture.InterceptVelocityThresh or not self.bot_in_front_of_ball(),
            'moving to Capture')

        # Revert to intercept
        self.add_transition(
            Capture.State.coarse_approach, Capture.State.intercept,
            lambda: main.ball().vel.mag() >= Capture.InterceptVelocityThresh and self.bot_in_front_of_ball(),
            'moving to intercept')

        self.add_transition(
            Capture.State.fine_approach, Capture.State.intercept,
            lambda: main.ball().vel.mag() >= Capture.InterceptVelocityThresh and self.bot_in_front_of_ball(),
            'moving to intercept')

        # Hook to Fine
        self.add_transition(
            Capture.State.coarse_approach, Capture.State.fine_approach,
            lambda: self.find_coarse_point().near_point(self.robot.pos, 0.2) or self.bot_near_ball(Capture.CoarseApproachDist),
            'moving to hook')

        # Hook to Fine
        self.add_transition(
            Capture.State.fine_approach, Capture.State.coarse_approach,
            lambda: not self.bot_near_ball(Capture.CoarseApproachDist),
            'moving to hook')

        #DELAY STATES
        self.add_transition(
            Capture.State.fine_approach, Capture.State.delay,
            lambda: evaluation.ball.robot_has_ball(self.robot),
            'has ball')

        self.add_transition(
            Capture.State.delay, Capture.State.fine_approach,
            lambda: not evaluation.ball.robot_has_ball(self.robot),
            'lost ball during delay')

        self.add_transition(
            Capture.State.delay, behavior.Behavior.State.completed,
            lambda: time.time() - self.start_time > Capture.DelayTime and
            evaluation.ball.robot_has_ball(self.robot),
            'delay before finish')

        self.lastApproachTarget = None
        self.faceBall = faceBall

    # Helper Functions
    def bot_to_ball(self):
        return main.ball().pos - self.robot.pos

    def bot_near_ball(self, distance):
        return (self.bot_to_ball().mag() < distance)

    def bot_in_front_of_ball(self):
        ball2bot = self.bot_to_ball() * -1
        return (ball2bot.normalized().dot(main.ball().vel) > Capture.InFrontOfBallCosOfAngleThreshold)

    def ball_moving_towards_bot(self):
        return (main.ball().pos).dist_to(self.robot.pos) < (main.ball().vel + main.ball().pos).dist_to(self.robot.pos)

    # calculates intercept point for the fast moving intercept state
    def find_intercept_point(self):
        return find_robot_intercept_point(self.robot)

    # returns intercept point for the slow moving capture states
    def find_capture_point(self):
        return find_robot_capture_point(self.robot)

    def find_coarse_point(self):
        return find_robot_coarse_point(self.robot)

    def execute_running(self):
        self.robot.set_planning_priority(planning_priority.CAPTURE)

        if (self.faceBall):
            self.robot.face(main.ball().pos)

    # sets move subbehavior
    def execute_intercept(self):
        self.robot.set_dribble_speed(self.dribbler_power)
        self.robot.disable_avoid_ball()
        pos = self.find_intercept_point()
        self.robot.move_to(pos)

    def on_enter_coarse_approach(self):
        self.lastApproachTarget = None

    def execute_coarse_approach(self):
        move_point = self.find_coarse_point()
        if (self.lastApproachTarget != None and (move_point - self.lastApproachTarget).mag() < 0.2):
            move_point = self.lastApproachTarget
        self.lastApproachTarget = move_point
        # don't hit the ball on accident
        if move_point.dist_to(main.ball().pos) < Capture.CoarseApproachAvoidBall + constants.Robot.Radius:
            self.robot.disable_avoid_ball()
        else:
            self.robot.set_avoid_ball_radius(Capture.CoarseApproachAvoidBall)

        self.robot.move_to(move_point)
        main.system_state().draw_circle(self.lastApproachTarget,
                                        constants.Ball.Radius,
                                        constants.Colors.White, "Capture")

    def on_exit_coarse_approach(self):
        self.lastApproachTarget is None

    def execute_fine_approach(self):
        self.robot.disable_avoid_ball()
        self.robot.set_dribble_speed(self.dribbler_power)

        # TODO(ashaw596): explain this math a bit
        bot2ball = (main.ball().pos - self.robot.pos).normalized()
        aproach = self.bot_to_ball() + bot2ball * Capture.FineApproachSpeed / 4 + main.ball().vel
        if (aproach.mag() > 1):
            aproach = aproach.normalized() * 1
        self.robot.set_world_vel(aproach)

    def on_enter_delay(self):
        self.start_time = time.time()

    def execute_delay(self):
        self.robot.disable_avoid_ball()
        self.robot.set_dribble_speed(self.dribbler_power)

    def role_requirements(self):
        reqs = super().role_requirements()
        reqs.require_kicking = True

        for r in role_assignment.iterate_role_requirements_tree_leaves(reqs):
            if main.ball().valid:
                if self.state == Capture.State.intercept:
                    reqs.cost_func = lambda r: robocup.Line(main.ball().pos, main.ball().pos + main.ball().vel * 10).dist_to(r.pos)
                else:
                    reqs.cost_func = lambda r: main.ball().pos.dist_to(r.pos)
        return reqs

# Robot based helper functions
# calculates intercept point for the fast moving intercept state
def find_robot_intercept_point(robot):
    if (robot is not None):
        passline = robocup.Line(main.ball().pos, main.ball().pos + main.ball().vel * 10)
        pos = passline.nearest_point(robot.pos) + (main.ball().vel * Capture.DampenMult)
        return pos
    else:
        return None

# calculates capture point for the slow or stationary fine approach state
def find_robot_capture_point(robot):
    if robot is None:
        return main.ball().pos

    approach_vec = approach_vector(robot)
    # sample every 5 cm in the -approach_vector direction from the ball
    pos = None

    for i in range(50):
        dist = i * 0.05
        pos = main.ball().pos + main.ball().vel + approach_vec * dist
        # how long will it take the ball to get there
        ball_time = evaluation.ball.rev_predict(main.ball().vel, dist)
        robotDist = (pos - robot.pos).mag() * 0.6
        bot_time = robocup.get_trapezoidal_time(robotDist, robotDist, 2.2, 1,
                                                robot.vel.mag(), 0)

        if bot_time < ball_time:
            break

    return pos

def find_robot_coarse_point(robot):
    pos = main.ball().pos + main.ball().vel
    move_point = pos

    if main.ball().vel.mag() > Capture.CoarseApproachVelocity:
        move_point = move_point + main.ball().vel * 0.4
        if pos.cross(robot.pos) <= 0:
            move_point.rotate(pos, math.pi/2)
        elif pos.cross(robot.pos) > 0:
            move_point.rotate(pos, -1 * (math.pi/2))
    else:
        move_point = main.ball().pos

    return move_point

def approach_vector(robot):
    if main.ball().vel.mag() > 0.05:
        # ball's moving, get on the side it's moving towards
        return main.ball().vel.normalized()
    else:
        return (robot.pos - main.ball().pos).normalized()
