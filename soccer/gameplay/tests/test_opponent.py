import unittest
import robocup
import main
import evaluation.opponent
import constants

class TestOpponent(unittest.TestCase):
    def __init__(self, *args, **kwargs):
        super(TestOpponent, self).__init__(*args, **kwargs)
        self.context = robocup.Context()

    def setUp(self):
        main.init(False)
        main.set_context(self.context)

        for robot in main.system_state().their_robots:
            robot.set_vis_for_testing(True)

        self.length = constants.Field.Length
        self.width = constants.Field.Width
        self.botRadius = constants.Robot.Radius

        self.center_y = self.length / 2
        self.right_side = self.width / 2
        self.left_side = -self.width / 2

        self.their_robots = main.system_state().their_robots[0:6]
        self.our_robots = main.system_state().our_robots[0:6]
        self.ball = main.system_state().ball
        self.ball.set_pos_for_testing(robocup.Point(0, 0))

        for our_robot, their_robot in zip(self.our_robots, self.their_robots):
            our_robot.set_vis_for_testing(False)
            their_robot.set_vis_for_testing(False)

        main.set_their_robots(main.system_state().their_robots[0:6])
        main.set_our_robots(main.system_state().our_robots[0:6])

    # Tests a point for get_closest_opponent
    # 
    # @param x: x coordinate of test point
    # @param y: y coordinate of test point
    #
    def get_closest_opponent(self,
                             x,
                             y,
                             directional_weight=0,
                             excluded_robots=[]):
        return evaluation.opponent.get_closest_opponent(
            robocup.Point(x, y), directional_weight, excluded_robots)

    # Tests num_on_offense
    #
    def num_on_offense(self):
        return evaluation.opponent.num_on_offense()

    # Set the location of a robot 
    # We must use this function so that the C++ can act on the robot location
    # 
    # @param a_bot: bot to change position
    # @param x: new x position of bot
    # @param y: new y position of bot
    #
    def set_bot_pos(self, a_bot, x, y):
        a_bot.set_vis_for_testing(True)
        a_bot.set_pos_for_testing(robocup.Point(x, y))

    # Set the location of many robot 
    # We must use this function so that the C++ can act on the robot location
    # 
    # @param bot_arr: bot_arr to change position
    # @param x: new x position of bots
    # @param y: new y position of bots
    #
    def set_bots_pos(self, bot_arr, x, y):
        for bot in bot_arr:
            self.set_bot_pos(bot, x, y)

    def test_num_on_offense_one_bot(self):
        their_bot1 = self.their_robots[0]

        # Enemy robots located at their goal, ball is at our goal
        self.set_bot_pos(their_bot1, 0, self.length)
        self.assertEqual(
            self.num_on_offense(), 0,
            "Enemy robots located at the enemy goal are considered on offense")

        # Halfway
        self.set_bot_pos(their_bot1, 0, self.center_y)
        self.assertEqual(self.num_on_offense(), 1)

        # Some locations on our side of the field
        self.set_bot_pos(their_bot1, 0, self.center_y / 2)
        self.assertEqual(self.num_on_offense(), 1)

        self.set_bot_pos(their_bot1, self.right_side, self.center_y)
        self.assertEqual(self.num_on_offense(), 1)

        self.set_bot_pos(their_bot1, self.left_side, self.center_y)
        self.assertEqual(self.num_on_offense(), 1)

        self.set_bot_pos(their_bot1, self.right_side, self.center_y / 2)
        self.assertEqual(self.num_on_offense(), 1)

        self.set_bot_pos(their_bot1, self.left_side, self.center_y / 2)
        self.assertEqual(self.num_on_offense(), 1)

        # Enemy robots located at our goal, ball is at our goal
        self.set_bot_pos(their_bot1, 0, 0)
        self.assertEqual(
            self.num_on_offense(), 1,
            "Enemy robots located at our goal are not considered on offense")

    def test_num_on_offense_multiple_bots_and_ball(self):
        their_bot1, their_bot2, their_bot3 = self.their_robots[0:3]

        # 3 robots are located at our goal, ball is at our goal
        bot_arr = [their_bot1, their_bot2, their_bot3]
        self.set_bots_pos(bot_arr, 0, 0)
        self.assertEqual(
            self.num_on_offense(), 3,
            "some enemy robots located at our goal are not considered on offense"
        )

        # Move one bot away
        self.set_bot_pos(their_bot3, 0, self.length)
        self.assertEqual(self.num_on_offense(), 2)

        # Move bots to their side of the field
        self.set_bots_pos(bot_arr, 0, self.length * 3 / 4)
        self.assertEqual(self.num_on_offense(), 0)

        # Test ball factor
        self.ball.set_pos_for_testing(robocup.Point(0, self.length * 3 / 4))
        self.assertEqual(self.num_on_offense(), 3)

        # Move one bot away
        self.set_bot_pos(their_bot3, 0, self.length)
        self.assertEqual(self.num_on_offense(), 2)

    def test_get_closest_opponent_one_bot(self):
        their_bot1 = self.their_robots[0]

        # bot at center of field
        self.set_bot_pos(their_bot1, 0, self.center_y)

        # Test a bunch of points
        self.assertEqual(self.get_closest_opponent(0, 0), their_bot1)

        self.assertEqual(self.get_closest_opponent(0, self.length), their_bot1)

        self.assertEqual(
            self.get_closest_opponent(0, self.center_y), their_bot1)

        self.assertEqual(
            self.get_closest_opponent(self.left_side, self.center_y),
            their_bot1)

        self.assertEqual(
            self.get_closest_opponent(self.right_side, self.center_y),
            their_bot1)

        # Test points around the bot
        self.assertEqual(
            self.get_closest_opponent(0, self.center_y + self.botRadius),
            their_bot1)
        self.assertEqual(
            self.get_closest_opponent(0, self.center_y - self.botRadius),
            their_bot1)
        self.assertEqual(
            self.get_closest_opponent(self.botRadius, self.center_y),
            their_bot1)
        self.assertEqual(
            self.get_closest_opponent(-self.botRadius, self.center_y),
            their_bot1)

    def test_get_closest_opponent_two_bots(self):
        their_bot1, their_bot2 = self.their_robots[0:2]

        # Bot 1 is at our goal
        self.set_bot_pos(their_bot1, 0, 0)

        # Bot 2 is at center of the field
        self.set_bot_pos(their_bot2, 0, self.center_y)

        # Test points around their_bot1
        self.assertEqual(
            self.get_closest_opponent(0, self.botRadius), their_bot1)
        self.assertEqual(
            self.get_closest_opponent(self.botRadius, 0), their_bot1)
        self.assertEqual(
            self.get_closest_opponent(-self.botRadius, 0), their_bot1)

        # Test points around their_bot2
        self.assertEqual(
            self.get_closest_opponent(0, self.center_y + self.botRadius),
            their_bot2)
        self.assertEqual(
            self.get_closest_opponent(0, self.center_y - self.botRadius),
            their_bot2)
        self.assertEqual(
            self.get_closest_opponent(self.botRadius, self.center_y),
            their_bot2)
        self.assertEqual(
            self.get_closest_opponent(-self.botRadius, self.center_y),
            their_bot2)

        # Test center sides
        self.assertEqual(
            self.get_closest_opponent(self.right_side, self.center_y),
            their_bot2)
        self.assertEqual(
            self.get_closest_opponent(self.left_side, self.center_y),
            their_bot2)

        # Test bottom corners
        self.assertEqual(
            self.get_closest_opponent(self.right_side, 0), their_bot1)
        self.assertEqual(
            self.get_closest_opponent(self.left_side, 0), their_bot1)

        # Test equidistant
        self.assertEqual(
            self.get_closest_opponent(0, self.center_y / 2), their_bot1)

    def test_get_closest_opponent_our_bot_and_exclusion(self):
        their_bot1, their_bot2 = self.their_robots[0:2]
        our_bot1 = self.our_robots[0]

        self.set_bot_pos(their_bot1, 0, 0)
        self.set_bot_pos(our_bot1, 0, self.center_y)

        # Test trivial
        self.assertEqual(
            self.get_closest_opponent(0, self.botRadius), their_bot1)

        # Test our_bot ignored
        self.assertEqual(
            self.get_closest_opponent(0, self.center_y + self.botRadius),
            their_bot1)

        self.set_bot_pos(their_bot2, 0, self.center_y)

        # Test trivial bot2
        self.assertEqual(
            self.get_closest_opponent(0, self.center_y + self.botRadius),
            their_bot2)

        # Test bot exclusion
        self.assertEqual(
            self.get_closest_opponent(
                0,
                self.center_y + self.botRadius,
                excluded_robots=[their_bot2]), their_bot1)

    def test_get_closest_opponent_dir_weight(self):
        their_bot1, their_bot2 = self.their_robots[0:2]

        self.set_bot_pos(their_bot1, 0, 0)
        self.set_bot_pos(their_bot2, 0, self.center_y)

        # Choose a point equidistant between two robots with direction weight 0. Should choose the first one in the array
        self.assertEqual(
            self.get_closest_opponent(0, self.center_y / 2, 0), their_bot1,
            "failed equidistant with direction_weight = 0 ")

        # Choose a point equidistant between two robots with direction weight 1. Should choose the robot closest to their goal.
        self.assertEqual(
            self.get_closest_opponent(0, self.center_y / 2), their_bot1,
            "failed to choose robot closer to their goal")

        # Test directional weight, so the robot at the center of the field should have higher weight
        self.assertEqual(
            self.get_closest_opponent(0, self.center_y / 2, .1), their_bot2,
            "failed direction_weight = .5")
