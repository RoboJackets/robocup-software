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


class Capture(single_robot_behavior.SingleRobotBehavior):
    # Speed in m/s at which a capture will be handled by coarse and fine approach instead of intercept
    InterceptVelocityThresh = 0.15

    # Multiplied by the speed of the ball to find a "dampened" point to move to during an intercept
    DampenMult = 0.0

    # The distance to transition from coarse approach to fine
    # TODO: The correct way to do this would be using our official max acceleration and current speed
    CoarseToFineApproachDistance = 0.3

    # The distance to avoid the  ball in coarse approach
    CoarseApproachAvoidBall = 0.1

    # Minimum speed (On top of ball speed) to move towards the ball
    FineApproachMinDeltaSpeed = 0.1

    # Proportional term on the distance error between ball and robot during fine approach
    # Adds to the fine approach speed
    FineApproachDistanceMultiplier = .1

    # How much of the ball speed to add to our approach speed
    FineApproachBallSpeedMultiplier = .8

    # Time in which to wait in delay state to confirm the robot has the ball
    DelayTime = 0.5

    # Default dribbler speed, can be overriden by self.dribbler_power
    # Sets dribbler speed during intercept and fine approach
    DribbleSpeed = 100

    # The minimum dot product result between the ball and the robot to count as the ball moving at the
    # robot
    InFrontOfBallCosOfAngleThreshold = 0.95

    DelaySpeed = 0.2

    DelayBallSpeedMultiplier = 1

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

        self.add_state(Capture.State.intercept,
                       behavior.Behavior.State.running)
        self.add_state(Capture.State.coarse_approach,
                       behavior.Behavior.State.running)
        self.add_state(Capture.State.fine_approach,
                       behavior.Behavior.State.running)
        self.add_state(Capture.State.delay,
                       behavior.Behavior.State.running)

        self.add_transition(behavior.Behavior.State.start,
                            Capture.State.intercept, lambda: True,
                            'immediately')

        self.add_transition(
            Capture.State.intercept,
            Capture.State.coarse_approach,
            lambda: main.ball().vel.mag() < Capture.InterceptVelocityThresh or
                    (main.ball().pos).dist_to(self.robot.pos) < (main.ball().vel + main.ball().pos).dist_to(self.robot.pos),
            'Moving to dampen')

        self.add_transition(
            Capture.State.coarse_approach,
            Capture.State.intercept,
            lambda: main.ball().vel.mag() >= Capture.InterceptVelocityThresh and
                    (main.ball().pos).dist_to(self.robot.pos) >= (main.ball().vel + main.ball().pos).dist_to(self.robot.pos),
            'Moving to intercept')

        self.add_transition(
            Capture.State.coarse_approach,
            Capture.State.fine_approach,
            lambda: (self.bot_in_front_of_ball() or
                    self.bot_near_ball(Capture.CoarseToFineApproachDistance)) and main.ball().valid,
            'dist to ball < threshold')

        self.add_transition(
            Capture.State.fine_approach,
            Capture.State.delay,
            lambda: self.robot.has_ball, #evaluation.ball.robot_has_ball(self.robot),
            'Has ball')

        self.add_transition(
            Capture.State.delay,
            behavior.Behavior.State.completed,
            lambda: time.time() - self.start_time > Capture.DelayTime and
                    self.robot.has_ball, #evaluation.ball.robot_has_ball(self.robot),
            'Delay before finish')

        self.add_transition(
            Capture.State.delay,
            Capture.State.fine_approach,
            lambda: not self.robot.has_ball, #not evaluation.ball.robot_has_ball(self.robot),
            'Lost ball during delay')

        self.add_transition(
            Capture.State.fine_approach, Capture.State.coarse_approach,
            lambda: not (self.bot_in_front_of_ball() or
                        self.bot_near_ball(Capture.CoarseToFineApproachDistance)) and
                    (not self.bot_near_ball(Capture.CoarseToFineApproachDistance * 1.5) or
                    not main.ball().pos),
            'Lost ball during delay')

        self.dribbler_power = Capture.DribbleSpeed

        self.lastApproachTarget = None
        self.faceBall = faceBall

    def bot_to_ball(self):
        return main.ball().pos - self.robot.pos

    def bot_near_ball(self, distance):
        return (self.bot_to_ball().mag() < distance)

    # Ball is moving towards us and will not stop before reaching us
    def bot_in_front_of_ball(self):
        ball2bot = self.bot_to_ball() * -1
        return (ball2bot.normalized().dot(main.ball().vel) > Capture.InFrontOfBallCosOfAngleThreshold) and \
               ((ball2bot).mag() < (evaluation.ball.predict_stop() - main.ball().pos).mag())

    # calculates intercept point for the fast moving intercept state
    def find_moving_intercept(self):
        return find_robot_intercept_point(self.robot)

    # returns intercept point for the slow moving capture states
    def find_intercept_point(self):
        return find_robot_capture_point(self.robot)

    def execute_running(self):
        self.robot.set_planning_priority(planning_priority.CAPTURE)

        if (self.faceBall):
            self.robot.face(main.ball().pos)

    # sets move subbehavior
    def execute_intercept(self):
        self.robot.set_dribble_speed(self.dribbler_power)
        pos = self.find_moving_intercept()
        self.robot.move_to(pos)

    def on_enter_coarse_approach(self):
        self.lastApproachTarget == None

    def execute_coarse_approach(self):
        pos = self.find_intercept_point()

        # Don't move our target if it's somewhat close to our old target
        if (self.lastApproachTarget != None and
            (pos - self.lastApproachTarget).mag() < 0.1):
            pos = self.lastApproachTarget

        self.lastApproachTarget = pos

        # don't hit the ball on accident
        if pos.dist_to(main.ball().pos) < Capture.CoarseApproachAvoidBall + constants.Robot.Radius:
            self.robot.disable_avoid_ball()
        else:
            self.robot.set_avoid_ball_radius(Capture.CoarseApproachAvoidBall)

        self.robot.move_to(pos)
        main.debug_drawer().draw_circle(self.lastApproachTarget,
                                        constants.Ball.Radius,
                                        constants.Colors.White, "Capture")

    def on_exit_coarse_approach(self):
        self.lastApproachTarget is None

    def execute_fine_approach(self):
        self.robot.disable_avoid_ball()
        self.robot.set_dribble_speed(self.dribbler_power)

        bot2ball_dir = (main.ball().pos - self.robot.pos).normalized()

        approach = self.bot_to_ball() * Capture.FineApproachDistanceMultiplier + \
                    bot2ball_dir * Capture.FineApproachMinDeltaSpeed + \
                    main.ball().vel * Capture.FineApproachBallSpeedMultiplier
        if (approach.mag() > 1):
            approach = approach.normalized() * 1
        self.robot.set_world_vel(approach)

    def on_enter_delay(self):
        self.start_time = time.time()

    def execute_delay(self):
        self.robot.disable_avoid_ball()
        self.robot.set_dribble_speed(self.dribbler_power)

        ball_dir = (main.ball().pos - self.robot.pos).normalized()
        if main.ball().vel.mag() < Capture.DelaySpeed:
            self.robot.set_world_vel(ball_dir*Capture.DelaySpeed)
        elif main.ball().vel.mag() < Capture.DelaySpeed:
            delay_speed = main.ball().vel.mag() * Capture.DelayBallSpeedMultiplier - Capture.DelaySpeed
            self.robot.set_world_vel(ball_dir*delay_speed)
        self.robot.face(main.ball().pos)

    def role_requirements(self):
        reqs = super().role_requirements()
        reqs.require_kicking = True
        # try to be near the ball
        if main.ball().valid:
            if main.ball().vel.mag() < self.InterceptVelocityThresh:
                reqs.destination_shape = main.ball().pos
            else:
                # TODO Make this less complicated and remove magic numbers
                reqs.destination_shape = robocup.Segment(main.ball().pos, main.ball().pos + main.ball().vel * 10)
        return reqs

# calculates intercept point for the fast moving intercept state
def find_robot_intercept_point(robot):
    if (robot is not None):
        passline = robocup.Line(
            main.ball().pos, main.ball().pos + main.ball().vel * 10)
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
        pos = main.ball().pos + approach_vec * dist
        # how long will it take the ball to get there
        ball_time = evaluation.ball.rev_predict(dist)
        robotDist = (pos - robot.pos).mag() * 0.6
        bot_time = robocup.get_trapezoidal_time(robotDist, robotDist, 2.2, 1,
                                                robot.vel.mag(), 0)

        if bot_time < ball_time:
            break

    return pos


def approach_vector(robot):
    if main.ball().vel.mag() > 0.25 \
       and robot.pos.dist_to(main.ball().pos) > 0.2:
        # ball's moving, get on the side it's moving towards
        return main.ball().vel.normalized()
    else:
        return (robot.pos - main.ball().pos).normalized()
