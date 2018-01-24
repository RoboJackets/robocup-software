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
		self.pos = robocup.Point(float(x), float(y))
		self.visible = True;

	def set_pos(self, x, y):
		self.pos = robocup.Point(float(x), float(y))

class TestPassing(unittest.TestCase):
	def __init__(self, *args, **kwargs):
		super(TestPassing, self).__init__(*args, **kwargs)
		self.config = robocup.Configuration.FromRegisteredConfigurables()
		self.system_state = robocup.SystemState()	
	
	def setUp(self):
		
		# self.their_robots = [bot1, bot2, bot3, bot4, bot5, bot6]
		# main.set_their_robots(self.their_robots

		main.init()
		main.set_system_state(self.system_state)

		for robot in main.system_state().their_robots:
			robot.set_vis_for_testing(True)

		self.our_robots = [Moc_Robot(0, 0), Moc_Robot(0, 0), Moc_Robot(0, 0), 
				Moc_Robot(0, 0), Moc_Robot(0, 0), Moc_Robot(0, 0)]
		main.set_our_robots(self.our_robots)

		main.set_ball(Moc_Ball(0, 0))

		

	# Set some of our robots position to a single point
	#
	# @param numBots: number of robots to send to point
	# @param x final x coordinate of robots
	# @param y final y coordinate of robots
	#
	def set_our_robot_pos(self, numBots, x, y):
		for i in range(numBots):
			self.our_robots[i].set_pos(x, y)
		main.set_our_robots(self.our_robots)

	# Set some of their robots position to a single point
	#
	# @param numBots: number of robots to send to point
	# @param x final x coordinate of robots
	# @param y final y coordinate of robots
	#
	def set_their_robot_pos(self, numBots, x, y):
		for i in range(numBots):
			self.their_robots[i].set_pos(x, y)
		main.set_their_robots(self.their_robots)

	def test_eval_pass(self):
		length = constants.Field.Length
		width = constants.Field.Width

		bot1, bot2, bot3 = main.system_state().their_robots[0:3]

		def run_function(x1, y1, x2, y2, excluded_robots=[]):
			return evaluation.passing.eval_pass(robocup.Point(x1, y1), robocup.Point(x2, y2), excluded_robots)

		fail = 0
		success = .8

		# Test a point passing to itself. 
		# Supposed to return 0. 
		# This is an edge case that we discussed to be okay. In the future, if we want a pass that starts and ends at the same point to be considered successful, we need to modify the eval_pass function. 
		# self.assertEqual(run_function(0, 0, 0, 0), fail)

		# Test a point passing to a close point. Should return maximal value as long as no robots block
		# self.assertEqual(run_function(0, 0, 0, 0.1), success)

		# Test a point passing to a far point. Should return maximal value as long as no robots block
		# self.assertEqual(run_function(0, 0, 0, length / 2), success)

		bot1.set_pos_for_testing(robocup.Point(0, length / 4))
		bot2.set_pos_for_testing(robocup.Point(constants.Robot.Radius, length / 4))
		bot3.set_pos_for_testing(robocup.Point(-constants.Robot.Radius, length / 4))

		# for robot in main.system_state().their_robots:
		# 	print("robot pos: ", robot.pos)

		self.assertEqual(run_function(0, 0, 0, length / 2), success)

		# self.assertEqual(run_function(0, 0, 0, length / 2), success)		

	
