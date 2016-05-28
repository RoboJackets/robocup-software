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


class TouchBall(skills.capture.Capture):
    def __init__(self):
        super().__init__(faceBall=False)

    # Move back so we hit the mouth, not the side
    AdjDist = constants.Robot.Radius * 2

    ## Override this to detect if the ball is directly in front of us
    def bot_in_front_of_ball(self):
        adjFactor = robocup.Point(
            math.cos(self.robot.angle) * -TouchBall.AdjDist,
            math.sin(self.robot.angle) * -TouchBall.AdjDist)
        return (self.robot.pos - adjFactor).dist_to(main.ball().pos) \
            < TouchBall.AdjDist + constants.Robot.Radius

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
