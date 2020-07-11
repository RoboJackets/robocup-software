import gameplay_test
from gameplay_test import testRobot
import robocup


class Example(gameplay_test.GameplayTest):
    def __init__(self):
        super().__init__()
        self.ballPosition = robocup.Point(1, 1)
        self.ballVelocity = robocup.Point(-5, -5)

        self.name = "Simple Example Test"

        # List of plays to enable e.g. ["stopped", "offense/basic_122"]
        # You can also add the name of a playbook file i.e.
        # ["comp2019.pbk", "testing/test_pivot_kick"]
        # If the list is empty, the enabled play list will remain as it was
        # before the test was run
        self.play_list = ["comp2019.pbk", "testing/test_pivot_kick"]

        self.ourRobots = [
            testRobot(robocup.Point(2, -.5)),
            testRobot(robocup.Point(2, .5))
        ]
        self.theirRobots = [
            testRobot(robocup.Point(-2, -.5)),
            testRobot(robocup.Point(-2, .5))
        ]
