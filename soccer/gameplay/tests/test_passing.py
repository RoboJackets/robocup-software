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
		main.init()
		main.set_system_state(self.system_state)

		for robot in main.system_state().their_robots:
			robot.set_vis_for_testing(True)

	def set_bot_pos(self, a_bot, x, y):
		a_bot.set_pos_for_testing(robocup.Point(x, y))

	def test_eval_pass(self):
		length = constants.Field.Length
		width = constants.Field.Width
		botRadius = constants.Robot.Radius

		bot1, bot2, bot3, bot4, bot5 = main.system_state().their_robots[0:5]
		# bot1, bot2, bot3, bot4, bot5 = main.system_state().our_robots[0:5]

		def run_function(x1, y1, x2, y2, excluded_robots=[]):
			return evaluation.passing.eval_pass(robocup.Point(x1, y1), robocup.Point(x2, y2), excluded_robots)

		fail = 0
		success = .8

		# Test a point passing to itself. 
		# Supposed to return 0. 
		# This is an edge case that we discussed to be okay. 
		# In the future, if we want a pass that starts and ends at the same point 
		# to be considered successful, we need to modify the eval_pass function. 
		self.assertEqual(run_function(0, 0, 0, 0), fail, "point pass to itself succeeded")

		# Test a point passing to a close point. Should be successful
		self.assertEqual(run_function(0, 0, 0, 0.1), success, "point pass to close point fail")

		# Test a point passing to a far point. Should be successful
		self.assertEqual(run_function(0, 0, 0, length / 2), success, "point pass to half the field fail")

		# Test a point passing to the end of the field
		self.assertEqual(run_function(0, 0, 0, length), success, "point pass to the end of the field fail")

		# If a robot is right in front of the shooter, the pass will fail
		self.set_bot_pos(bot1, 0, botRadius * 2)
		self.assertEqual(run_function(0, 0, 0, length / 2), fail, "pass with robot in front of the shooter success")

		# Test excluded_robots
		self.assertEqual(run_function(0, 0, 0, length / 2, [bot1]), success, "test excluded_robots")

		# Robot halfway between shooter and goal point. 
		self.set_bot_pos(bot1, 0, length / 4)
		self.assertGreater(run_function(0, 0, 0, length / 2), fail, "robot halfway between shooter and goal point fail")
		self.assertLess(run_function(0, 0, 0, length / 2), success, "robot halfway between shooter and goal point success")

		

	
