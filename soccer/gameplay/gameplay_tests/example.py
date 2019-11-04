import gameplay_test
from gameplay_test import testRobot
import robocup

class Example(gameplay_test.GameplayTest):

    def __init__(self):
        super().__init__()
        self.ballPosition = robocup.Point(1, 1)
        self.ballVelocity = robocup.Point(-5, -5)

        self.name = "Simple Example Test"

        self.play_list = ["offense/basic_122","testing/test_pivot_kick"]

        self.ourRobots = [testRobot(2, -.5), testRobot(2, .5)]
        self.theirRobots = [testRobot(-2, -.5), testRobot(-2, .5)]
