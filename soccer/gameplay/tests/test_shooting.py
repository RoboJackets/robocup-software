import unittest 
import robocup
import main
import evaluation.shooting
import constants

class Moc_Robot:
	def __init__(self, x, y):
		self.pos = robocup.Point(x, y)
		self.visible = True;

	def set_pos(self, x, y):
		self.pos = robocup.Point(x, y)

class TestShooting(unittest.TestCase):
	def __init__(self, *args, **kwargs):
		super(TestShooting, self).__init__(*args, **kwargs)

	# def setUp(self):
	# 	main.init()
	# 	self.robots = [Moc_Robot(0, 0)]
	# 	main.set_their_robots(self.robots)

	def test_eval_shot(self):
		length = constants.Field.Length
		width = constants.Field.Width

		def run_function(x, y):
			return evaluation.shooting.eval_shot(robocup.Point(x, y))

		self.assertGreater(run_function(0, length), 0.99)
		self.assertGreater(run_function(0, 3 * length / 4), 0.99)

		self.assertGreater(run_function(width / 4, 3 * length / 4), 0.99)
		self.assertGreater(run_function(-width / 4, 3 * length / 4), 0.99)

		# print("width: " ,width)
		# print("length: ", length)
		# print(run_function(width / 2, 3 * length / 4))