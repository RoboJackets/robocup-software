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
		main.set_system_state(robocup.SystemState())

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

		bot1 = robocup.OpponentRobot(1)
		bot2 = robocup.OpponentRobot(2)
		bot3 = robocup.OpponentRobot(3)
		bot4 = robocup.OpponentRobot(4)
		bot5 = robocup.OpponentRobot(5)
		bot6 = robocup.OpponentRobot(6)

		def run_function(x1, y1, x2, y2, excluded_robots=[]):
			return evaluation.passing.eval_pass(robocup.Point(x1, y1), robocup.Point(x2, y2), excluded_robots)

		fail = 0
		success = .8

		# Test a point passing to itself. 
		# Supposed to return 0. 
		# This is an edge case that we discussed to be okay. In the future, if we want a pass that starts and ends at the same point to be considered successful, we need to modify the eval_pass function. 
		# self.assertEqual(run_function(0, 0, 0, 0), fail)

		# Test a point passing to a close point. Should return maximal value
		# self.assertEqual(run_function(0, 0, 0, 0.1), success)

		# print("length / 4: {}".format(length / 4))
		# print("length / 2: {}".format(length / 2))

		# Test a point passing to a far point. Should return maximal value
		# self.assertEqual(run_function(0, length / 4, 0, length / 2), success)

		# self.set_our_robot_pos(1, 0, length / 2)
		# self.set_their_robot_pos(1, 0, length / 2)

		# print("starting")

		bot1.set_pos_for_testing(robocup.Point(2, length / 4))
		bot2.set_pos_for_testing(robocup.Point(5, length / 4))
		bot3.set_pos_for_testing(robocup.Point(4, length / 4))
		bot4.set_pos_for_testing(robocup.Point(0, length / 4))
		bot5.set_pos_for_testing(robocup.Point(0, length / 4))




		# main.set_system_state(robocup.SystemState())
		# self.their_robots[0].set_pos(0, length / 4 + constants.Robot.Radius * 2)
		# self.their_robots[1].set_pos(constants.Robot.Radius * 2 , length / 2)
		# self.their_robots[2].set_pos(constants.Robot.Radius * -3 , length / 2 - constants.Robot.Radius)
		# self.their_robots[3].set_pos(.1, length / 4 + constants.Robot.Radius)
		# main.set_their_robots(self.their_robots)

		# self.our_robots[0].set_pos(0, length / 4 + constants.Robot.Radius * 2)
		# self.our_robots[1].set_pos(constants.Robot.Radius * 2 , length / 2)
		# self.our_robots[2].set_pos(constants.Robot.Radius * -3 , length / 2 - constants.Robot.Radius)
		# self.our_robots[3].set_pos(.1, length / 4 + constants.Robot.Radius)
		# main.set_our_robots(self.our_robots)
		self.assertEqual(run_function(0, 0, 0, length / 2), success)		

		# print("Ending")
		# self.set_our_robot_pos(6, 0, length / 2)
		# self.set_their_robot_pos(6, 0, length / 2)
		# self.assertEqual(run_function(0, length / 2, 0, length / 2 + .01), .8)

	
