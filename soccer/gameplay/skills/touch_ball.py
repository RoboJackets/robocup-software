import single_robot_behavior
import behavior
from enum import Enum
import main
import constants
import robocup


## Touchball Class
# A touchball is a simplified capture behavior, that simply lets the ball
# roll towards the robot until it hits it in the mouth
#
# This class does NOT handle turning towards the ball, as it is designed to be
# used with a class that will try to shoot it at a target.
class TouchBall(single_robot_behavior.SingleRobotBehavior):

    # tunable config values
    DribbleSpeed = 0

    class State(Enum):
        course_approach = 1

    # Move back so we hit the mouth, not the side.
    AdjDist = constants.Robot.Radius

    ## TouchBall Constructor
    # useful for reflecting/bouncing moving ballls.
    def __init__(self):
        super().__init__(continuous=False)

        self.add_state(TouchBall.State.course_approach,
                       behavior.Behavior.State.running)

        self.add_transition(behavior.Behavior.State.start,
                            TouchBall.State.course_approach, lambda: True,
                            'immediately')

        self.add_transition(TouchBall.State.course_approach,
                            behavior.Behavior.State.completed,
                            lambda: self.robot.has_ball(), 'Ball got hit!')

        self.add_transition(TouchBall.State.course_approach,
                            behavior.Behavior.State.failed,
                            lambda: not main.ball().valid,  # TODO fail properly
                            'ball was lost')

    # normalized vector pointing from the ball to the point the robot should get to in course_aproach
    def approach_vector(self):
        return main.ball().vel.normalized()

    def find_intercept_point(self, adjusted=True):
        approach_vec = self.approach_vector()

        adjFactor = robocup.Point.direction(self.robot.angle) \
                    * -TouchBall.AdjDist
        robotPos = self.robot.pos - adjFactor

        # multiply by a large enough value to cover the field.
        approach_line = robocup.Line(
            main.ball().pos,
            main.ball().pos + approach_vec * constants.Field.Length)
        pos = approach_line.nearest_point(robotPos)

        if adjusted:
            pos += adjFactor

        return pos

    def execute_course_approach(self):
        # don't hit the ball on accident
        pos = self.find_intercept_point()
        self.robot.move_to(pos)

    def role_requirements(self):
        reqs = super().role_requirements()
        reqs.require_kicking = True
        # try to be near the ball
        if main.ball().valid:
            reqs.destination_shape = main.ball().pos
        return reqs
