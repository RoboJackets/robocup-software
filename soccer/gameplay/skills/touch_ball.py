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
    DribbleSpeed = 0
    FineApproachSpeed = 0.2
    # The amount of seconds to look in the future when
    # trying to actualy hit the eball. TODO: use maxaccel to find this.
    HitAdjust = 1

    class State(Enum):
        course_approach = 1
        hit_ball = 2

    # Move back so we hit the mouth, not the side
    AdjDist = constants.Robot.Radius * 2

    ## TouchBall Constructor
    # useful for reflecting/bouncing moving ballls.
    def __init__(self ):
        super().__init__(continuous=False)

        self.add_state(TouchBall.State.course_approach,
                       behavior.Behavior.State.running)
        self.add_state(TouchBall.State.hit_ball,
                       behavior.Behavior.State.running)

        self.add_transition(behavior.Behavior.State.start,
                            TouchBall.State.course_approach, lambda: True,
                            'immediately')

        self.add_transition(
            TouchBall.State.course_approach, TouchBall.State.hit_ball,
            lambda: (self.ball_in_front_of_bot()) and main.ball().valid,
            'dist to ball < threshold')

        self.add_transition(TouchBall.State.hit_ball,
                            behavior.Behavior.State.completed,
                            lambda: self.robot.has_ball(), 'has ball')

        self.add_transition(
            TouchBall.State.hit_ball, behavior.Behavior.State.failed,
            lambda: not (self.ball_in_front_of_bot()) and not main.ball().pos,
            'ball went into goal')

        self.lastApproachTarget = None


    ## Override this to detect if the ball is directly in front of us
    def ball_in_front_of_bot(self):
        adjFactor = robocup.Point.direction(self.robot.angle) \
                    *  -TouchBall.AdjDist
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
    def find_intercept_point(self, adjusted=True):
        approach_vec = self.approach_vector()

        adjFactor = robocup.Point.direction(self.robot.angle) \
                    *  -TouchBall.AdjDist
        robotPos = self.robot.pos - adjFactor

        # multiply by a large enough value to cover the field.
        approach_line = robocup.Line(main.ball().pos, main.ball().pos +
                                     approach_vec * constants.Field.Length)
        pos = approach_line.nearest_point(robotPos)

        if adjusted:
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

    def execute_hit_ball(self):
        self.robot.disable_avoid_ball()
        self.robot.set_dribble_speed(TouchBall.DribbleSpeed)
        self.robot.move_to_direct(main.ball().pos +
                                  (main.ball().vel * (1 / TouchBall.HitAdjust)))

    def role_requirements(self):
        reqs = super().role_requirements()
        reqs.require_kicking = True
        # try to be near the ball
        if main.ball().valid:
            reqs.destination_shape = main.ball().pos
        return reqs
