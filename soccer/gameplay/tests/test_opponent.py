import unittest 
import robocup
import main
import evaluation.opponent
import constants

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

	# Set some robots position to a single point
	# modify robots[start] -> modify[end] position
	#
	# @param start starting index to modify
	# @param end last index to modify
	# @param x final x coordinate of robots
	# @param y final y coordinate of robots
	#
	def set_robot_pos(self, start, end, x, y):
		for i in range(start, end + 1):
			self.robots[i].set_pos(x, y)
		main.set_their_robots(self.robots)

	def test_num_on_offense(self):
		self.set_robot_pos(0, 2, 0, constants.Field.Length)
		val = evaluation.opponent.num_on_offense()
		print(val)
		print(main.ball().pos)