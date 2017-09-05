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
		self.our_robots = [Moc_Robot(0, 0), Moc_Robot(0, 0), Moc_Robot(0, 0), 
				Moc_Robot(0, 0), Moc_Robot(0, 0), Moc_Robot(0, 0)]
		main.set_their_robots(self.our_robots)

	def set_robot_pos(self, x, y):
		for robot in self.our_robots:
			robot.set_pos(x, y)

	def test_space_coeff_at_pos(self):
		def run_function(x, y):
			return evaluation.field.space_coeff_at_pos(robocup.Point(x, y))
		
		self.assertTrue(run_function(0, 0) == 1)
		self.set_robot_pos(0, constants.Field.Length)
		self.assertTrue(run_function(0, 0) == 0) 
