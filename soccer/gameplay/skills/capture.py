import single_robot_behavior
import behavior
from enum import Enum
import main
import evaluation
import constants
import role_assignment
import robocup
import planning_priority


class Capture(single_robot_behavior.SingleRobotBehavior):

    # tunable config values
    CourseApproachErrorThresh = 0.8
    CourseApproachDist = 0.4
    CourseApproachAvoidBall = 0.10

    # Default dribbler speed, can be overriden by self.dribbler_power
    DribbleSpeed = 100
    FineApproachSpeed = 0.1

    InFrontOfBallCosOfAngleThreshold = 0.95

    class State(Enum):
        course_approach = 1
        fine_approach = 2

    ## Capture Constructor
    # faceBall - If false, any turning functions are turned off,
    # useful for using capture to reflect/bounce moving ballls.
    def __init__(self, faceBall=True):
        super().__init__(continuous=False)

        self.add_state(Capture.State.course_approach,
                       behavior.Behavior.State.running)
        self.add_state(Capture.State.fine_approach,
                       behavior.Behavior.State.running)

        self.add_transition(behavior.Behavior.State.start,
                            Capture.State.course_approach, lambda: True,
                            'immediately')

        self.add_transition(
            Capture.State.course_approach,
            Capture.State.fine_approach, lambda: (self.bot_in_front_of_ball(
            ) or self.bot_near_ball(Capture.CourseApproachDist)) and main.ball(
            ).valid, 'dist to ball < threshold')

        self.add_transition(
            Capture.State.fine_approach,
            behavior.Behavior.State.completed, lambda: self.bot_near_ball(
                constants.Robot.Radius + constants.Ball.Radius), 'has ball')

        self.add_transition(
            Capture.State.fine_approach, Capture.State.course_approach, lambda:
            not (self.bot_in_front_of_ball() or self.bot_near_ball(
                Capture.CourseApproachDist)) and (not self.bot_near_ball(
                    Capture.CourseApproachDist * 1.5) or not main.ball().pos),
            'ball went into goal')

        self.dribbler_power = Capture.DribbleSpeed

        self.lastApproachTarget = None
        self.faceBall = faceBall

    def bot_to_ball(self):
        return main.ball().pos - self.robot.pos

    def bot_near_ball(self, distance):
        return (self.bot_to_ball().mag() < distance)

    def bot_in_front_of_ball(self):
        ball2bot = self.bot_to_ball() * -1
        return (ball2bot.normalized().dot(main.ball().vel) > Capture.InFrontOfBallCosOfAngleThreshold) and \
                ((ball2bot).mag() < (evaluation.ball.predict_stop(main.ball().pos, main.ball().vel) - main.ball().pos).mag())

    # normalized vector pointing from the ball to the point the robot should get to in course_aproach

    def find_intercept_point(self):
        return find_robot_intercept_point(self.robot)

    def execute_running(self):
        self.robot.set_planning_priority(planning_priority.CAPTURE)

        if (self.faceBall):
            self.robot.face(main.ball().pos)

    def on_enter_course_approach(self):
        self.lastApproachTarget == None

    def execute_course_approach(self):
        pos = self.find_intercept_point()

        if (self.lastApproachTarget != None and
            (pos - self.lastApproachTarget).mag() < 0.1):
            pos = self.lastApproachTarget

        self.lastApproachTarget = pos

        # don't hit the ball on accident
        if pos.dist_to(main.ball(
        ).pos) < Capture.CourseApproachAvoidBall + constants.Robot.Radius:
            self.robot.disable_avoid_ball()
        else:
            self.robot.set_avoid_ball_radius(Capture.CourseApproachAvoidBall)

        self.robot.move_to(pos)
        main.system_state().draw_circle(self.lastApproachTarget,
                                        constants.Ball.Radius,
                                        constants.Colors.White, "Capture")

    def on_exit_course_approach(self):
        self.lastApproachTarget is None

    def execute_fine_approach(self):
        self.robot.disable_avoid_ball()
        self.robot.set_dribble_speed(self.dribbler_power)

        # TODO(ashaw596): explain this math a bit
        bot2ball = (main.ball().pos - self.robot.pos).normalized()
        multiplier = 1.5
        aproach = self.bot_to_ball(
        ) * multiplier + bot2ball * Capture.FineApproachSpeed / 4 + main.ball(
        ).vel
        if (aproach.mag() > 1):
            aproach = aproach.normalized() * 1
        self.robot.set_world_vel(aproach)

    def role_requirements(self):
        reqs = super().role_requirements()
        reqs.require_kicking = True
        # try to be near the ball
        if main.ball().valid:
            reqs.cost_func = lambda r: reqs.position_cost_multiplier * find_robot_intercept_point(
                r).dist_to(r.pos)

        return reqs


def find_robot_intercept_point(robot):
    if robot is None:
        return main.ball().pos
    approach_vec = approach_vector(robot)
    # sample every 5 cm in the -approach_vector direction from the ball
    pos = None
    for i in range(50):
        dist = i * 0.05
        pos = main.ball().pos + approach_vec * dist
        # how long will it take the ball to get there
        ball_time = evaluation.ball.rev_predict(main.ball().vel, dist)
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
