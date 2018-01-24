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

class TestPassing(unittest.TestCase):
	def __init__(self, *args, **kwargs):
		super(TestPassing, self).__init__(*args, **kwargs)

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

