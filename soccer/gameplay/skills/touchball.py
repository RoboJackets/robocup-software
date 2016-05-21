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
        super().__init__(onlyApproach=True)

    AdjustDistance = 0.25

    ## A touch is different from a capture in that we should try to keep our
    # distance from the ball if possible, and move forward to hit the ball at
    # the last minute.
    # To do this, let's move the intercept point capture found back a bit.
    def find_intercept_point(self):
        # Capture's interception point.
        pos = super().find_intercept_point()

        pos += robocup.Point(math.cos(self.robot.angle) * -TouchBall.AdjustDistance, math.sin(self.robot.angle) * -TouchBall.AdjustDistance)
        return pos
