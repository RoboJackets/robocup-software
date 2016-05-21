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

    adjDist = constants.Robot.Radius * 2 # Move back so the

    ## Override so this has no impact on state transitions
    def bot_in_front_of_ball(self):
        return False


    ## A touch is different from a capture in that we should try to keep our
    # distance from the ball if possible, and move forward to hit the ball at
    # the last minute.
    # To do this, let's move the intercept point capture found back a bit.
    #
    # In addition, lets try to keep this point stable by choosing the closest
    # point, instead of the point that we can reach in time closest to the ball
    def find_intercept_point(self):
        approach_vec = self.approach_vector()

        adjFactor = robocup.Point(math.cos(self.robot.angle) * -TouchBall.adjDist, math.sin(self.robot.angle) * -TouchBall.adjDist)

        # sample every 5 cm in the -approach_vector direction from the ball
        pos = None
        # Adjust where this algorithm thinks we are by the adjust factor.
        robotPos = self.robot.pos - adjFactor
        finalPos = None
        minBotTime = float('inf')
        for i in range(50):
            dist = i * 0.05
            pos = main.ball().pos + approach_vec * dist
            # how long will it take the ball to get there
            ball_time = evaluation.ball.rev_predict(main.ball().vel, dist)
            robotDist = (pos - (robotPos)).mag() * 0.6
            bot_time = robocup.get_trapezoidal_time(robotDist, robotDist, 2.2,
                                                    1, self.robot.vel.mag(), 0)

            if bot_time < minBotTime or finalPos == None:
                minBotTime = bot_time
                finalPos = pos

        # Push found point back a bit to have the ball hit our mouth,
        # instead of our side.
        finalPos += adjFactor

        return finalPos
