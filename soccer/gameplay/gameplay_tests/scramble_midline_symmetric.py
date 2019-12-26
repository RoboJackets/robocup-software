import gameplay_test
from gameplay_test import testRobot
import robocup
import constants


class ScrambleMidlineSymmetric(gameplay_test.GameplayTest):
    def __init__(self):
        super().__init__()


        center_field = robocup.Point(0, constants.Field.Length / 2)
        zero_zero = robocup.Point(0,0)

        self.ballPosition = center_field
        self.ballVelocity = zero_zero

        self.name = "Scramble Midline Symmetric"

        #I've left this using a test playbook as this should be a full stack test as is
        self.play_list = ["test_book.pbk"]

        self.ourRobots = [testRobot(zero_zero)]
                #testRobot(center_field + robocup.Point(0.25, 0.5)),
                #testRobot(center_field + robocup.Point(0.25, -0.5))]


        self.theirRobots = []#[testRobot(robocup.Point(-2, -.5)), testRobot(robocup.Point(-2, .5))]
