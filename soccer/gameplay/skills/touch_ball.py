import single_robot_behavior
import behavior
from enum import Enum
import main
import evaluation
import constants
import role_assignment
import robocup
import skills.capture
import math


class TouchBall(single_robot_behavior.SingleRobotBehavior):

    # tunable config values
    CourseApproachDist = 0.4
    CourseApproachAvoidBall = 0.10
    DribbleSpeed = 100
    FineApproachSpeed = 0.2

    class State(Enum):
        course_approach = 1
        fine_approach = 2

    # Move back so we hit the mouth, not the side
    AdjDist = constants.Robot.Radius * 2

    ## TouchBall Constructor
    # useful for reflecting/bouncing moving ballls.
    def __init__(self ):
        super().__init__(continuous=False)

        self.add_state(TouchBall.State.course_approach,
                       behavior.Behavior.State.running)
        self.add_state(TouchBall.State.fine_approach,
                       behavior.Behavior.State.running)

        self.add_transition(behavior.Behavior.State.start,
                            TouchBall.State.course_approach, lambda: True,
                            'immediately')

        self.add_transition(
            TouchBall.State.course_approach, TouchBall.State.fine_approach,
            lambda: (self.bot_in_front_of_ball() or self.bot_near_ball(TouchBall.CourseApproachDist)) and main.ball().valid,
            'dist to ball < threshold')

        self.add_transition(TouchBall.State.fine_approach,
                            behavior.Behavior.State.completed,
                            lambda: self.robot.has_ball(), 'has ball')

        self.add_transition(
            TouchBall.State.fine_approach, behavior.Behavior.State.failed,
            lambda: not (self.bot_in_front_of_ball() or self.bot_near_ball(TouchBall.CourseApproachDist)) and (not self.bot_near_ball(TouchBall.CourseApproachDist * 1.5) or not main.ball().pos),
            'ball went into goal')

        self.lastApproachTarget = None


    def bot_to_ball(self):
        return main.ball().pos - self.robot.pos

    def bot_near_ball(self, distance):
        return (self.bot_to_ball().mag() < distance)


    ## Override this to detect if the ball is directly in front of us
    def bot_in_front_of_ball(self):
        adjFactor = robocup.Point(
            math.cos(self.robot.angle) * -TouchBall.AdjDist,
            math.sin(self.robot.angle) * -TouchBall.AdjDist)
        return (self.robot.pos - adjFactor).dist_to(main.ball().pos) \
            < TouchBall.AdjDist + constants.Robot.Radius

    # normalized vector pointing from the ball to the point the robot should get to in course_aproach
    def approach_vector(self):
        if main.ball().vel.mag() > 0.25 \
            and self.robot.pos.dist_to(main.ball().pos) > 0.2:
            # ball's moving, get on the side it's moving towards
            return main.ball().vel.normalized()
        else:
            return (self.robot.pos - main.ball().pos).normalized()


    ## A touch is different from a capture in that we should try to keep our
    # distance from the ball if possible, and move forward to hit the ball at
    # the last minute.
    # To do this, let's move the intercept point capture found back a bit.
    #
    # In addition, lets try to keep this point stable by choosing the closest
    # point, instead of the point that we can reach in time closest to the ball
    def find_intercept_point(self):
        approach_vec = self.approach_vector()

        adjFactor = robocup.Point.direction(self.robot.angle) \
                    *  -TouchBall.AdjDist
        robotPos = self.robot.pos - adjFactor

        # multiply by a large enough value to cover the field.
        approach_line = robocup.Line(main.ball().pos, main.ball().pos +
                                     approach_vec * constants.Field.Length)
        pos = approach_line.nearest_point(robotPos)

        pos += adjFactor
        return pos


    def execute_running(self):
        # make sure teammates don't bump into us
        self.robot.shield_from_teammates(constants.Robot.Radius * 2.0)

    def on_enter_course_approach(self):
        self.lastApproachTarget == None

    def execute_course_approach(self):
        # don't hit the ball on accident
        self.robot.set_avoid_ball_radius(TouchBall.CourseApproachAvoidBall)
        pos = self.find_intercept_point()


        if (self.lastApproachTarget != None and
            (pos - self.lastApproachTarget).mag() < 0.1):
            self.robot.move_to(self.lastApproachTarget)
            main.system_state().draw_circle(self.lastApproachTarget,
                                            constants.Ball.Radius,
                                            constants.Colors.White, "TouchBall")
        else:
            main.system_state().draw_circle(pos, constants.Ball.Radius,
                                            constants.Colors.White, "TouchBall")
            self.robot.move_to(pos)
            self.lastApproachTarget = pos

    def on_exit_course_approach(self):
        self.lastApproachTarget == None

    def execute_fine_approach(self):
        self.robot.disable_avoid_ball()
        self.robot.set_dribble_speed(TouchBall.DribbleSpeed)

        # TODO(ashaw596): explain this math a bit
        bot2ball = (main.ball().pos - self.robot.pos).normalized()
        multiplier = 1.5
        aproach = self.bot_to_ball(
        ) * multiplier + bot2ball * TouchBall.FineApproachSpeed / 4 + main.ball(
        ).vel
        if (aproach.mag() > 1):
            aproach = aproach.normalized() * 1
        self.robot.set_world_vel(aproach)

    def role_requirements(self):
        reqs = super().role_requirements()
        reqs.require_kicking = True
        # try to be near the ball
        if main.ball().valid:
            reqs.destination_shape = main.ball().pos
        return reqs
