import unittest
import robocup
import main
import evaluation.shooting
import constants

class TestShooting(unittest.TestCase):
    def __init__(self, *args, **kwargs):
        super(TestShooting, self).__init__(*args, **kwargs)
        self.context = robocup.Context()

    def setUp(self):
        main.set_context(self.context)

        for robot in main.system_state().their_robots:
            robot.set_vis_for_testing(True)

        self.length = constants.Field.Length
        self.width = constants.Field.Width
        self.botRadius = constants.Robot.Radius

        self.their_robots = main.system_state().their_robots[0:6]
        self.our_robots = main.system_state().our_robots[0:6]

        self.success = 1
        self.failure = 0

    def eval_shot(self, x, y, excluded_robots=[]):
        return evaluation.shooting.eval_shot(
            robocup.Point(x, y), excluded_robots)

    # Set the location of a robot 
    # We must use this function so that the C++ can act on the robot location
    # 
    # @param a_bot: bot to change position
    # @param x: new x position of bot
    # @param y: new y position of bot
    #
    def set_bot_pos(self, a_bot, x, y):
        a_bot.set_pos_for_testing(robocup.Point(x, y))

    def test_eval_shot_clear_field(self):
        # Test shot from their goal (to their goal)
        self.assertEqual(self.eval_shot(0, self.length), self.success)

        # Test shot from 3/4 of the field 
        self.assertEqual(self.eval_shot(0, 3 * self.length / 4), self.success)

        # Test shot from just past the center of the field
        self.assertEqual(
            self.eval_shot(0, self.length / 2 + self.botRadius), self.success)

        # Test shot from some angles
        self.assertGreater(
            self.eval_shot(self.width / 4, 3 * self.length / 4), 0.99)
        self.assertGreater(
            self.eval_shot(-self.width / 4, 3 * self.length / 4), 0.99)

        self.assertGreater(
            self.eval_shot(self.width / 4, self.length / 2 + self.botRadius),
            0.95)
        self.assertGreater(
            self.eval_shot(-self.width / 4, self.length / 2 + self.botRadius),
            0.95)

        # Corner shots should not have 100% chance to hit the goal
        self.assertEqual(
            self.eval_shot(self.width / 2, self.length), self.failure)

        # Just inside the corner will not be 100%
        self.assertLess(
            self.eval_shot(self.width / 2 - self.botRadius,
                           self.length - self.botRadius), 0.2)

    @unittest.skip("Skip Problematic Cases")
    def test_eval_shot_problem_cases(self):
        # This location is very outside the field but we consider it successful
        # KickEvaluator does not check bounds
        self.assertEqual(self.eval_shot(0, 2 * self.length), self.failure)

        # Does not allow shot from midfield
        # This case is okay because we cannot score from our side of the field
        self.assertEqual(self.eval_shot(0, self.length / 2), self.success)

    def test_eval_shot_with_bots_and_exclusion(self):
        their_bot1 = self.their_robots[0]
        our_bot1 = self.our_robots[0]

        shooting_pos = 3 * self.length / 4

        # Test enemy bot right in front of shooting position
        self.set_bot_pos(their_bot1, 0, shooting_pos + self.botRadius)
        self.assertLess(self.eval_shot(0, shooting_pos), 0.05)

        # Test shot works when bot excluded
        self.assertEqual(
            self.eval_shot(0, shooting_pos, [their_bot1]), self.success)

        # Test friendly bot in front of shooting position
        self.set_bot_pos(our_bot1, 0, shooting_pos + self.botRadius)
        self.assertEqual(
            self.eval_shot(0, shooting_pos, [their_bot1]), self.success)

        # Test bot somewhere between shooting position and goal
        self.set_bot_pos(their_bot1, 0, shooting_pos + self.botRadius * 3)
        self.assertLess(self.eval_shot(0, shooting_pos), self.success)
        self.assertGreater(self.eval_shot(0, shooting_pos), self.failure)
        self.assertEqual(
            self.eval_shot(0, shooting_pos, [their_bot1]), self.success)
