import unittest
import main
import robocup
import evaluation.field
import constants

class Moc_Robot:
	def __init__(self, x, y):
		self.pos = robocup.Point(x, y)
		self.visible = True;

class TestField(unittest.TestCase):
	def __init__(self, *args, **kwargs):
		super(TestField, self).__init__(*args, **kwargs)

	def setUp(self):
		self.our_robots = [Moc_Robot(0, 0), Moc_Robot(0, 0), Moc_Robot(0, 0), 
				Moc_Robot(0, 0), Moc_Robot(0, 0), Moc_Robot(0, 0)]
		# main.set_our_robots(self.our_robots);

	def test_space_coeff_at_pos(self):
		self.assertTrue(evaluation.field.space_coeff_at_pos(robocup.Point(0, 0), excluded_robots=[], robots=self.our_robots) == 1)