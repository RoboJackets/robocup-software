import unittest 
import robocup
import main
import evaluation.opponent
import constants

class Moc_Ball:
	def __init__(self, x, y):
		self.pos = robocup.Point(x, y)

class Moc_Robot:
	def __init__(self, x, y):
		self.pos = robocup.Point(x, y)
		self.visible = True;

	def set_pos(self, x, y):
		self.pos = robocup.Point(x, y)

class TestOpponent(unittest.TestCase):
	def __init__(self, *args, **kwargs):
		super(TestOpponent, self).__init__(*args, **kwargs)
		self.system_state = robocup.SystemState()

	def setUp(self):
		main.init(False)
		main.set_system_state(self.system_state)

		for robot in main.system_state().their_robots:

			robot.set_vis_for_testing(True)

		self.their_robots = main.system_state().their_robots[0:6]
		self.our_robots = main.system_state().our_robots[0:6]

		main.set_their_robots(main.system_state().their_robots[0:6])
		main.set_our_robots(main.system_state().our_robots[0:6])
		main.set_ball(main.system_state().ball)

	# Set some robots position to a single point
	#
	# @param numBots: number of robots to send to point
	# @param x final x coordinate of robots
	# @param y final y coordinate of robots
	#
	def set_robot_pos(self, numBots, x, y):
		for i in range(numBots):
			self.their_robots[i].set_pos_for_testing(robocup.Point(x, y))

	def test_num_on_offense(self):
		length = constants.Field.Length
		width = constants.Field.Width

		# Easier than typing out whole function
		def run_function():
			return evaluation.opponent.num_on_offense()

		# Enemy robots located at their goal, ball is at our goal
		self.set_robot_pos(6, 0, length)
		self.assertEqual(run_function(), 0, "Enemy robots located at the enemy goal are considered on offense")

		# Enemy robots located at our goal, ball is at our goal
		self.set_robot_pos(6, 0, 0)
		self.assertEqual(run_function(), 6, "Enemy robots located at our goal are not considered on offense")

		# 3 robots are located at our goal, ball is at our goal
		self.set_robot_pos(3, 0, length)
		self.assertEqual(run_function(), 3, "some enemy robots located at our goal are not considered on offense")

		# Enemy robots are on their side of the field, ball is located close to them
		self.set_robot_pos(6, 0, length * 3 / 4)
		main.ball().set_pos_for_testing(robocup.Point(0, length * 3 / 4))
		self.assertEqual(run_function(), 6, "Enemy robots located near the ball are not considered on offense")
		
		# Enemy robots are on our side of the field, ball is located on the other side of the field
		self.set_robot_pos(6, 0, length / 4)
		self.assertEqual(run_function(), 6, "Enemy robots located on our side of the field are not considered on offense")

	def test_get_closest_opponent(self):
		length = constants.Field.Length
		width = constants.Field.Width

		# Tests a point for get_closest_opponent
		# 
		# @param x: x coordinate of test point
		# @param y: y coordinate of test point
		#
		def test_point(x, y, directional_weight=1, excluded_robots=[]):
			return evaluation.opponent.get_closest_opponent(robocup.Point(x, y), directional_weight, excluded_robots)

		# All robots are at (0, 0), check that we get the first robot in the array
		self.assertEqual(test_point(0, 0), main.their_robots()[0], "fail trivial case")

		# All robots are at (0, 0), check that function works when the point is far
		self.assertEqual(test_point(0, length / 2), main.their_robots()[0], "all robots at our goal, test center of field, should choose first robot in array")

		main.their_robots()[1].set_pos_for_testing(robocup.Point(0, length / 2))
		
		# One robot at center of field, choose right center of field
		self.assertEqual(test_point(width / 2, length / 2), main.their_robots()[1], "failed to assert smaller distance")

		# Choose a point equidistant between two robots with direction weight 0. Should choose the first one in the array
		self.assertEqual(test_point(0, length / 4, 0), main.their_robots()[0], "failed equidistant with direction_weight = 0 ")

		# Choose a point equidistant between two robots with direction weight 1. Should choose the robot closest to their goal.
		self.assertEqual(test_point(0, length / 4), main.their_robots()[1], "failed to choose robot closer to their goal")

		# Test directional weight, so the robot at the center of the field should have higher weight
		self.assertEqual(test_point(0, length / 4, .5), main.their_robots()[1], "failed direction_weight = .5")

		# Test excluded robots
		self.assertEqual(test_point(0, length / 2, excluded_robots=[main.their_robots()[0]]), main.their_robots()[1], "failed to exclude robots")

