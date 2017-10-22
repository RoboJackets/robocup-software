import unittest 
import robocup
import main
import evaluation.shooting

class Moc_Robot:
	def __init__(self, x, y):
		self.pos = robocup.Point(x, y)
		self.visible = True;

	def set_pos(self, x, y):
		self.pos = robocup.Point(x, y)

class TestShooting(unittest.TestCase):
	def __init__(self, *args, **kwargs):
		super(TestShooting, self).__init__(*args, **kwargs)

	def setUp(self):
		main.init()
		self.robots = [Moc_Robot(0, 0)]
		main.set_their_robots(self.robots)

	def test_eval_shot(self):
		val = "x"
		val = evaluation.shooting.eval_shot(robocup.Point(0,2))
		# Ask Joe why this doesn't work :(
		print(val)