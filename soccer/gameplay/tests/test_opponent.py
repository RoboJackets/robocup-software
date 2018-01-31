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

	def setUp(self):
		self.robots = [Moc_Robot(0, 0), Moc_Robot(0, 0), Moc_Robot(0, 0), 
				Moc_Robot(0, 0), Moc_Robot(0, 0), Moc_Robot(0, 0)]
		main.set_their_robots(self.robots)
		main.set_ball(Moc_Ball(0, 0))

	# Set some robots position to a single point
	#
	# @param numBots: number of robots to send to point
	# @param x final x coordinate of robots
	# @param y final y coordinate of robots
	#
	def set_robot_pos(self, numBots, x, y):
		for i in range(numBots):
			self.robots[i].set_pos(x, y)
		main.set_their_robots(self.robots)

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
		main.set_ball(Moc_Ball(0, length * 3 / 4))
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

		def printPoints():
			robots = main.their_robots()
			print()
			for i in range(len(robots)):
				print("Robot {} pos:({}, {})".format(i, robots[i].pos.x, robots[i].pos.y))


		# All robots are at (0, 0), check that we get the first robot in the array
		self.assertEqual(test_point(0, 0), main.their_robots()[0])

		# All robots are at (0, 0), check that function works when the point is far
		self.assertEqual(test_point(0, length / 2), main.their_robots()[0])

		main.their_robots()[1].set_pos(0, length / 2)
		
		# One robot at center of field, choose right center of field
		self.assertEqual(test_point(width / 2, length / 2), main.their_robots()[1])

		# Choose a point equidistant between two robots with direction weight 0. Should choose the first one in the array
		self.assertEqual(test_point(0, length / 4, 0), main.their_robots()[0])

		self.assertEqual(test_point(0, length / 4), main.their_robots()[1])

		# Test directional weight, so the robot at the center of the field should have higher weight
		self.assertEqual(test_point(0, length / 4, .5), main.their_robots()[1])

		# 5 robots are at (0, 0), one robot is closer to the center. 
		# Check this robot is returned when we test the center of the field. 
		main.their_robots()[1].set_pos(0, length / 4)
		
		# This does not return what it should because function is broken
		self.assertEqual(test_point(0, length / 2), main.their_robots()[1])

		# Test excluded robots
		self.assertEqual(test_point(0, length / 2, excluded_robots=[main.their_robots()[0]]), main.their_robots()[1])

