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
	# @param x x coordinate of point
	# @param y y coordinate of point
	# 
	def set_robots_around_pos(self, x, y, rad):
		self.robots[0].set_pos(x - constants.Robot.Radius * rad, y - constants.Robot.Radius * rad)
		self.robots[1].set_pos(x - constants.Robot.Radius * rad, y + constants.Robot.Radius * rad)
		self.robots[2].set_pos(x, y + constants.Robot.Radius * rad)
		self.robots[3].set_pos(x + constants.Robot.Radius * rad, y - constants.Robot.Radius * rad)
		self.robots[4].set_pos(x + constants.Robot.Radius * rad, y)
		self.robots[5].set_pos(x, y - constants.Robot.Radius * rad)
		main.set_their_robots(self.robots)

	def test_space_coeff_at_pos(self):
		width = constants.Field.Width
		length = constants.Field.Length

		msg = "evaluation.field.space_coeff_at_pos not functioning as intended: "

		# Easier than writing out whole function
		def run_function(x, y):
			return evaluation.field.space_coeff_at_pos(robocup.Point(x, y))
		
		# both robots and position are at (0, 0)
		self.assertTrue(run_function(0, 0) == 1, msg = msg + "Position occupied by robots is considered open")

		# Robots and position are on opposite goals of the field
		self.set_robot_pos(0, length)
		self.assertTrue(run_function(0, 0) == 0, msg = msg + "Position devoid of robots is not considered open") 

		# Robots are bottom right corner, check bottom left position 
		self.set_robot_pos(width / 2, 0) 
		self.assertTrue(run_function(width * -1 / 2, 0) == 0, msg = msg + "Postion devoid of robots is not considered open")

		# 6 Robots are located around position, distance around ~.9 from position
		self.set_robots_around_pos(0, length / 2, 5)
		self.assertTrue(run_function(0, length / 2) >= 0, msg="Robots located around position considered to be open. Check constant inside function")



