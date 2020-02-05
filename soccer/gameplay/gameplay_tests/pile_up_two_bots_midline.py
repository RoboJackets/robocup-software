import gameplay_test
from gameplay_test import testRobot
import robocup
import constants


class PileUpTwoBotsMidline(gameplay_test.GameplayTest):
    def __init__(self):
        super().__init__()

        center_pos = robocup.Point(0, constants.Field.Length /
                                   2) + robocup.Point(0, 1)
        zero_zero = robocup.Point(0, 0)
        opposing_goal = robocup.Point(0, constants.Field.Length)

        self.ballPosition = center_pos
        self.ballVelocity = zero_zero

        self.name = "Scramble Midline Symmetric"

        #I've left this using a test playbook as this should be a full stack test as is
        self.play_list = ["test_book.pbk"]

        self.ourRobots = [
            testRobot(zero_zero),
            testRobot(self.ballPosition + robocup.Point(0, -0.09)),
            testRobot(center_pos + robocup.Point(0.5, -0.75)),
            testRobot(center_pos + robocup.Point(-0.5, -0.75)),
            testRobot(center_pos + robocup.Point(0, -1.7)),
            testRobot(center_pos + robocup.Point(0, -2.1))
        ]

        self.theirRobots = [
            testRobot(opposing_goal),
            testRobot(self.ballPosition + robocup.Point(0, 0.09), angle=180),
            testRobot(center_pos + robocup.Point(0.5, 0.75)),
            testRobot(center_pos + robocup.Point(-0.5, 0.75)),
            testRobot(center_pos + robocup.Point(0, 1.7)),
            testRobot(center_pos + robocup.Point(0, 2.1))
        ]
