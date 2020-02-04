import gameplay_test
from gameplay_test import testRobot
import robocup
import constants


class ScrambleMidlineSymmetric(gameplay_test.GameplayTest):
    def __init__(self):
        super().__init__()

        center_field = robocup.Point(0, constants.Field.Length / 2)
        zero_zero = robocup.Point(0, 0)
        opposing_goal = robocup.Point(0, constants.Field.Length)

        self.ballPosition = center_field
        self.ballVelocity = zero_zero

        self.name = "Scramble Midline Symmetric"

        #I've left this using a test playbook as this should be a full stack test as is
        self.play_list = ["test_book.pbk"]

        self.ourRobots = [
            testRobot(zero_zero),
            testRobot(center_field + robocup.Point(0, -0.75)),
            testRobot(center_field + robocup.Point(0.5, -0.75)),
            testRobot(center_field + robocup.Point(-0.5, -0.75)),
            testRobot(center_field + robocup.Point(0, -1.7)),
            testRobot(center_field + robocup.Point(0, -2.1))
        ]

        self.theirRobots = [
            testRobot(opposing_goal),
            testRobot(center_field + robocup.Point(0, 0.75)),
            testRobot(center_field + robocup.Point(0.5, 0.75)),
            testRobot(center_field + robocup.Point(-0.5, 0.75)),
            testRobot(center_field + robocup.Point(0, 1.7)),
            testRobot(center_field + robocup.Point(0, 2.1))
        ]
