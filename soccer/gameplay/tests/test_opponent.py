import unittest 
import robocup
import main
import evaluation.opponent
import constants

class TestOpponent(unittest.TestCase):
	def __init__(self, *args, **kwargs):
		super(TestOpponent, self).__init__(*args, **kwargs)
		self.system_state = robocup.SystemState()

	def setUp(self):
		main.init(False)
		main.set_system_state(self.system_state)

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

		for our_robot, their_robot in zip(self.our_robots, self.their_robots):
			our_robot.set_vis_for_testing(False)
			their_robot.set_vis_for_testing(False)

		main.set_their_robots(main.system_state().their_robots[0:6])
		main.set_our_robots(main.system_state().our_robots[0:6])
		main.set_ball(main.system_state().ball)


	# Tests a point for get_closest_opponent
	# 
	# @param x: x coordinate of test point
	# @param y: y coordinate of test point
	#
	def get_closest_opponent(self, x, y, directional_weight=1, excluded_robots=[]):
		return evaluation.opponent.get_closest_opponent(robocup.Point(x, y), directional_weight, excluded_robots)

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

	def test_num_on_offense_one_bot(self):
		their_bot1 = self.their_robots[0]

		# Enemy robots located at their goal, ball is at our goal
		self.set_bot_pos(their_bot1, 0, self.length)
		self.assertEqual(self.num_on_offense(), 0, "Enemy robots located at the enemy goal are considered on offense")

		# Enemy robots located at our goal, ball is at our goal
		self.set_bot_pos(their_bot1, 0, 0)
		self.assertEqual(self.num_on_offense(), 1, "Enemy robots located at our goal are not considered on offense")

		self.set_bot_pos(their_bot1, 0, self.center_y)
		self.assertEqual(self.num_on_offense(), 1, "Enemy robots located at our goal are not considered on offense")

		

	@unittest.skip("for now")		
	def test_num_on_offense_other(self):
		# 3 robots are located at our goal, ball is at our goal
		self.set_robot_pos(3, 0, length)
		self.assertEqual(self.num_on_offense(), 3, "some enemy robots located at our goal are not considered on offense")

		# Enemy robots are on their side of the field, ball is located close to them
		self.set_robot_pos(6, 0, length * 3 / 4)
		main.ball().set_pos_for_testing(robocup.Point(0, length * 3 / 4))
		self.assertEqual(self.num_on_offense(), 6, "Enemy robots located near the ball are not considered on offense")
		
		# Enemy robots are on our side of the field, ball is located on the other side of the field
		self.set_robot_pos(6, 0, length / 4)
		self.assertEqual(self.num_on_offense(), 6, "Enemy robots located on our side of the field are not considered on offense")

	def test_get_closest_opponent_one_bot(self):
		their_bot1 = self.their_robots[0]

		# bot at center of field
		self.set_bot_pos(their_bot1, 0, self.center_y)
		# their_bot1.set_pos_for_testing(robocup.Point(0, self.center_y))

		# Test a bunch of points
		self.assertEqual(self.get_closest_opponent(0, 0), their_bot1)

		self.assertEqual(self.get_closest_opponent(0, self.length), their_bot1)

		self.assertEqual(self.get_closest_opponent(0, self.center_y), their_bot1)

		self.assertEqual(self.get_closest_opponent(self.left_side, self.center_y), their_bot1)

		self.assertEqual(self.get_closest_opponent(self.right_side, self.center_y), their_bot1)

		# Test points around the bot
		self.assertEqual(self.get_closest_opponent(0, self.center_y + self.botRadius), their_bot1)
		self.assertEqual(self.get_closest_opponent(0, self.center_y - self.botRadius), their_bot1)
		self.assertEqual(self.get_closest_opponent(self.botRadius, self.center_y), their_bot1)
		self.assertEqual(self.get_closest_opponent(-self.botRadius, self.center_y), their_bot1)


	def test_get_closest_opponent_two_bots(self):
		their_bot1, their_bot2 = self.their_robots[0:2]

		self.set_bot_pos(their_bot1, 0, 0)

		self.set_bot_pos(their_bot2, 0, self.center_y)

		# Test points around their_bot1
		self.assertEqual(self.get_closest_opponent(0, self.botRadius), their_bot1)
		self.assertEqual(self.get_closest_opponent(self.botRadius, 0), their_bot1)
		self.assertEqual(self.get_closest_opponent(-self.botRadius, 0), their_bot1)

		# Test points around their_bot2
		self.assertEqual(self.get_closest_opponent(0, self.center_y + self.botRadius), their_bot2)
		self.assertEqual(self.get_closest_opponent(0, self.center_y - self.botRadius), their_bot2)
		self.assertEqual(self.get_closest_opponent(self.botRadius, self.center_y), their_bot2)
		self.assertEqual(self.get_closest_opponent(-self.botRadius, self.center_y), their_bot2)

		# Test center sides
		self.assertEqual(self.get_closest_opponent(self.right_side, self.center_y), their_bot2)
		self.assertEqual(self.get_closest_opponent(self.left_side, self.center_y), their_bot2)

		# Test bottom corners
		self.assertEqual(self.get_closest_opponent(self.right_side, 0), their_bot1)
		self.assertEqual(self.get_closest_opponent(self.left_side, 0), their_bot1)

		# Test equidistant
		self.assertEqual(self.get_closest_opponent(0, self.center_y / 2), their_bot2)

	def test_get_closest_opponent_our_bot_and_exclusion(self):
		their_bot1, their_bot2 = self.their_robots[0:2]
		our_bot1 = self.our_robots[0]

		self.set_bot_pos(their_bot1, 0, 0)
		self.set_bot_pos(our_bot1, 0, self.center_y)

		# Test trivial
		self.assertEqual(self.get_closest_opponent(0, self.botRadius), their_bot1)

		# Test our_bot ignored
		self.assertEqual(self.get_closest_opponent(0, self.center_y + self.botRadius), their_bot1)

		self.set_bot_pos(their_bot2, 0, self.center_y)

		# Test trivial bot2
		self.assertEqual(self.get_closest_opponent(0, self.center_y + self.botRadius), their_bot2)

		# Test bot exclusion
		self.assertEqual(self.get_closest_opponent(0, self.center_y + self.botRadius, excluded_robots=[their_bot2]), their_bot1)


	def test_get_closest_opponent_dir_weight(self):
		their_bot1, their_bot2 = self.their_robots[0:2]

		self.set_bot_pos(their_bot1, 0, 0)
		self.set_bot_pos(their_bot2, 0, self.center_y)

		# Choose a point equidistant between two robots with direction weight 0. Should choose the first one in the array
		self.assertEqual(self.get_closest_opponent(0, self.center_y / 2, 0), their_bot1, "failed equidistant with direction_weight = 0 ")

		# Choose a point equidistant between two robots with direction weight 1. Should choose the robot closest to their goal.
		self.assertEqual(self.get_closest_opponent(0, self.center_y / 2), their_bot2, "failed to choose robot closer to their goal")

		# Test directional weight, so the robot at the center of the field should have higher weight
		self.assertEqual(self.get_closest_opponent(0, self.center_y / 2, .1), their_bot2, "failed direction_weight = .5")
