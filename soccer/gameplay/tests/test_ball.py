import unittest
import main
import robocup
import evaluation.field
import constants

class Moc_Ball:
	def __init__(self, x, y):
		self.pos = robocup.Point(x, y)
		self.vel = robocup.Point(0, 0)

	def set_pos(self, x, y):
		self.pos = robocup.Point(x, y)

	def set_vel(self, x, y):
		self.vel = robocup.Point(x, y)

class TestBall(unittest.TestCase):
	def __init__(self, *args, **kwargs):
		super(TestBall, self).__init__(*args, **kwargs)

	def setUp(self):
		main.set_ball(Moc_Ball(0, constants.Field.Length / 2))

	def test_is_moving_towards_our_goal(self):
		width = constants.Field.Width
		length = constants.Field.Length

		msg = "evaluation.ball.is_moving_towards_our_goal() isn't functioning correctly: "

		# Easier than writing out whole function
		def test_function():
			return evaluation.ball.is_moving_towards_our_goal()

		# Test velocity = 0
		self.assertFalse(test_function(), msg + "function returns true when ball isn't moving")
		
		# Test ball going straight to goal from center
		main.ball().set_vel(0, -1)
		self.assertTrue(test_function(), msg + "function returns false when ball is going towards goal")
		
		# Test ball going opposite to goal from center
		main.ball().set_vel(0, 1)
		self.assertFalse(test_function(), msg + "function returns true when ball is going away from goal")

		# Test ball going towards goal from right side of field
		main.ball().set_pos(width / 4, length / 4)
		main.ball().set_vel(-1, -1)
		self.assertTrue(test_function(), msg + "function returns false when ball is going towards goal")

		# Test ball going towards the goal from left side of field
		main.ball().set_pos(-width / 4, length / 4)
		main.ball().set_vel(1, -1)
		self.assertTrue(test_function(), msg + "function returns false when ball is going towards goal")

	def test_is_in_our_goalie_zone(self):
		print("Hello World")