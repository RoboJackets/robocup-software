import unittest
import robocup
import main
import evaluation.opponent
import constants

class TestPassing(unittest.TestCase):
    def __init__(self, *args, **kwargs):
        super(TestPassing, self).__init__(*args, **kwargs)
        self.context = robocup.Context()

    def setUp(self):
        main.set_context(self.context)

        self.length = constants.Field.Length
        self.width = constants.Field.Width
        self.center_y = self.length / 2
        self.right_side = self.width / 2
        self.left_side = -self.width / 2
        self.botRadius = constants.Robot.Radius

        self.failure = 0
        self.success = .8

        self.their_robots = main.system_state().their_robots
        self.our_robots = main.system_state().our_robots

        for robot in main.system_state().their_robots:
            robot.set_vis_for_testing(True)

    # Returns eval_pass
    def eval_pass(self, x1, y1, x2, y2, excluded_robots=[]):
        return evaluation.passing.eval_pass(
            robocup.Point(x1, y1), robocup.Point(x2, y2), excluded_robots)

    # Set the location of a robot 
    # We must use this function so that the C++ can act on the robot location
    # 
    # @param a_bot: bot to change position
    # @param x: new x position of bot
    # @param y: new y position of bot
    #
    def set_bot_pos(self, a_bot, x, y):
        a_bot.set_pos_for_testing(robocup.Point(x, y))

    @unittest.skip("Skip Probematic Cases")
    def test_eval_pass_problem_cases(self):
        # Test a point passing to itself. 
        # Supposed to return 0. 
        # This is an edge case that we discussed to be okay. 
        # In the future, if we want a pass that starts and ends at the same point 
        # to be considered successful, we need to modify the eval_pass function. 
        self.assertEqual(
            self.eval_pass(0, 0, 0, 0), self.failure,
            "point pass to itself fail")

        # Test passing outside the fields. 
        # WindowEvaluator in eval_pass doesn't check outside the field
        self.assertEqual(
            self.eval_pass(0, 0, 0, 2 * self.length), self.success,
            "pass to the end of the field fail")

    def test_eval_pass_clear_field(self):
        # Test a point passing to a close point. Should be successful
        self.assertEqual(
            self.eval_pass(0, 0, 0, 0.1), self.success,
            "pass to close point fail")

        # Test a point passing to a far point. Should be successful
        self.assertEqual(
            self.eval_pass(0, 0, 0, self.center_y), self.success,
            "pass to half the field fail")

        # Test a point passing to the end of the field
        self.assertEqual(
            self.eval_pass(0, 0, 0, self.length), self.success,
            "pass to the end of the field fail")

        # Test a point passing from bottom left to top right of field
        self.assertEqual(
            self.eval_pass(self.left_side, 0, self.right_side, self.length),
            self.success, "pass from bottom left to top right fail")

        # Test horizontal pass across the center of the field
        self.assertEqual(
            self.eval_pass(self.left_side, self.center_y, self.right_side,
                           self.center_y), self.success,
            "horizontal pass across center of field")

    def test_eval_pass_with_bots_and_exclusion(self):
        their_bot1 = self.their_robots[0]
        our_bot1 = self.our_robots[0]

        passing_dest = self.center_y

        # If a robot is right in front of the shooter, the pass will fail
        self.set_bot_pos(their_bot1, 0, self.botRadius * 2)
        self.assertEqual(
            self.eval_pass(0, 0, 0, passing_dest), self.failure,
            "pass with robot in front of the shooter success")

        # Test excluded_robots
        self.assertEqual(
            self.eval_pass(0, 0, 0, passing_dest, [their_bot1]), self.success,
            "fail excluded_robots")

        # If a friendly robot is right in front of the shooter, the pass will still work
        self.set_bot_pos(our_bot1, 0, self.botRadius * 2)
        self.assertEqual(
            self.eval_pass(0, 0, 0, passing_dest, [their_bot1]), self.success,
            "fail excluded_robots")

        # Robot halfway between shooter and goal point. 
        self.set_bot_pos(their_bot1, 0, self.center_y / 2)
        self.assertGreater(
            self.eval_pass(0, 0, 0, passing_dest), self.failure,
            "robot halfway between shooter and goal point considered complete failure"
        )
        self.assertLess(
            self.eval_pass(0, 0, 0, passing_dest), self.success,
            "robot halfway between shooter and goal point considered complete success"
        )
        self.assertEqual(
            self.eval_pass(0, 0, 0, passing_dest, [their_bot1]), self.success,
            "fail excluded_robots")
