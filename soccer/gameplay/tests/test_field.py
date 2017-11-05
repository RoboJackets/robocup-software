import unittest
import main
import robocup
import evaluation.field
import constants

class Moc_Robot:
	def __init__(self, x, y):
		self.pos = robocup.Point(x, y)
		self.visible = True;

	def set_pos(self, x, y):
		self.pos = robocup.Point(x, y)

class TestField(unittest.TestCase):
	def __init__(self, *args, **kwargs):
		super(TestField, self).__init__(*args, **kwargs)

	def setUp(self):
		self.robots = [Moc_Robot(0, 0), Moc_Robot(0, 0), Moc_Robot(0, 0), 
				Moc_Robot(0, 0), Moc_Robot(0, 0), Moc_Robot(0, 0)]
		main.set_their_robots(self.robots)

	# Set 6 robots position to a single point
	#
	# @param x final x coordinate of robots
	# @param y final y coordinate of robots
	# 
	def set_robot_pos(self, x, y):
		for robot in self.robots:
			robot.set_pos(x, y)
		main.set_their_robots(self.robots)

	# Positions 6 robots around a point but not on it
	#
	# @param x: x coordinate of point
	# @param y: y coordinate of point
	# @param rad: distance of robots from point
	def set_robots_around_pos(self, x, y, rad):
		dist_from_point = constants.Robot.Radius * rad
		self.robots[0].set_pos(x - dist_from_point, y - dist_from_point)
		self.robots[1].set_pos(x - dist_from_point, y + dist_from_point)
		self.robots[2].set_pos(x, y + dist_from_point)
		self.robots[3].set_pos(x + dist_from_point, y - dist_from_point)
		self.robots[4].set_pos(x + dist_from_point, y)
		self.robots[5].set_pos(x, y - dist_from_point)
		main.set_their_robots(self.robots)

	def test_space_coeff_at_pos(self):
		width = constants.Field.Width
		length = constants.Field.Length

		# Easier than writing out whole function
		# Finds space_coeff_at_pos for a point
		#
		# @param x: x coordinate of point
		# @param y: y coordinate of point
		def run_function(x, y):
			return evaluation.field.space_coeff_at_pos(robocup.Point(x, y))
		
		# both robots and position are at (0, 0)
		self.assertAlmostEqual(run_function(0, 0), 1, msg = "Position occupied by robots is considered open")

		# Robots and position are on opposite goals of the field
		self.set_robot_pos(0, length)
		self.assertAlmostEqual(run_function(0, 0), 0, msg = "Position devoid of robots is not considered open") 

		# Robots are bottom right corner, check bottom left position 
		self.set_robot_pos(width / 2, 0) 
		self.assertAlmostEqual(run_function(width * -1 / 2, 0), 0, msg = "Postion devoid of robots is not considered open")

		# 6 Robots are located around position, distance around ~.9 from position
		self.set_robots_around_pos(0, length / 2, 5)
		self.assertGreater(run_function(0, length / 2), 0, msg="Robots located around position considered to be open. Check constant inside function")

	def test_field_pos_coeff_at_pos(self):

		# Easier than writing out whole function
		# Finds field_pos_coeff_at_pos for a point
		#
		# @param x: x coordinate of point
		# @param y: y coordinate of point
		# @param center: How much to weight being close to the 'center of the field'
		# @param dist: How much to weight being close to the opponents goal
		# @param angl: How much to weight the angle between the robot and the goal (In turn, how small the goal is)
		# @param attack: Whether attacking their goal
		def run_function(x, y, center, dist, angl, attack=True):
			return evaluation.field.field_pos_coeff_at_pos(robocup.Point(x, y), center, dist, angl, attack)

		width = constants.Field.Width
		length = constants.Field.Length

		#########################################
		## Test distance to center line factor ##
		#########################################

		# Test center of the field returns 1
		num = run_function(0, length / 2, 1 ,0 ,0)
		self.assertAlmostEqual(num, 1, msg = "center of the field doesn't return 1 for distance to center factor")

		# Test halfway between center and edge returns not 0 or 1
		num = run_function(width / 4, length / 2, 1, 0, 0)
		self.assertGreater(num, 0, "position between edge and center returns 1 or 0 for distance to center factor")
		self.assertLess(num, 1, "position between edge and center returns 1 or 0 for distance to center factor")

		# Test right edge of the field returns 0		
		num = run_function(width / 2, length / 2, 1, 0 ,0)
		self.assertAlmostEqual(num, 0, msg = "edge of the field doesn't return 0 for distance to center factor")

		##################################
		## Test distance to goal factor ##
		##################################

		# Test position close to goal returns 1
		num = run_function(0, length, 0, 1, 0)
		self.assertAlmostEqual(num, 1, msg = "position close to goal returns not 1 for distance to goal factor")

		# Test center of field returns not 0 or 1
		num = run_function(0, length / 2, 0, 1, 0)
		self.assertGreater(num, 0, "position in center should not return 0 or 1 for distance to goal factor")
		self.assertLess(num, 1, "position in center should not return 0 or 1 for distance to goal factor")

		# Test other side of field returns 0
		num = run_function(0, 0, 0, 1 ,0)
		self.assertAlmostEqual(num, 0, msg = "other side of field doesn't return 0 for distance to goalfactor")

		# Test switching sides returns 1 for same position
		num = run_function(0, 0, 0, 1, 0, False)
		self.assertAlmostEqual(num, 1, msg = "Switching sides doesn't return 1 for distance to goal factor")

		###############################
		## Test angle to goal factor ##
		###############################

		# Test front of goal returns 1
		num = run_function(0, length / 2, 0, 0, 1)
		self.assertAlmostEqual(num, 1, msg = "in front of the goal doesn't return 1 for angle to goal factor")

		# Test 45 angle returns not 0 or 1
		num = run_function(width / 2, length / 2, 0 ,0 ,1)
		self.assertGreater(num, 0, "45 degree angle returns 1 or 0 for angle to goal factor")
		self.assertLess(num, 1, "45 degree angle returns 1 or 0 for angle to goal factor")

		# Test corner of field returns 0
		num = run_function(width / 2, length, 0, 0, 1)
		self.assertAlmostEqual(num, 0, msg = "corner of the field returns not 0 for angle to goal factor")

		# Test corner of field returns not 0 or 1 when switching sides
		num = run_function(width / 2, length, 0, 0, 1, False)
		self.assertGreater(num, 0, "switching sides doesn't work for angle to goal factor")
		self.assertLess(num, 1, "switching sides doesn't work for angle to goal factor")


