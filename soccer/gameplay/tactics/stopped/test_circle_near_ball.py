import unittest
import main
import behavior
import robocup
import standard_play
import play
import math
import tactics.stopped.circle_near_ball
import constants

class Moc_Ball:
	def __init__(self, x, y):
		self.pos = robocup.Point(x, y)

class TestCircleNearBall(unittest.TestCase):
	def __init__(self, *args, **kwargs):
		super(TestCircleNearBall, self).__init__(*args, **kwargs)
		#self.config = robocup.Configuration.FromRegisteredConfigurables()
		#ball = Moc_Ball()

	def setUp(self):
		self.circle_near_ball = tactics.stopped.circle_near_ball.CircleNearBall()
		#self.system_state = robocup.SystemState()
	
	def test_get_circle_points(self):
		print("\n")
		main.set_ball(Moc_Ball(0, constants.Field.Length / 2))

		print("ball pos: " + str(main.ball().pos))
		print("Circle 1: " + str(self.circle_near_ball.get_circle_points(6)))
		main.set_ball(Moc_Ball(2.5, constants.Field.Length / 2))
		print("ball pos: " + str(main.ball().pos))
		print("Circle 2: " + str(self.circle_near_ball.get_circle_points(6)))
	# def test_execute_completed(self):

	def test_normalize_angle(self):
		print("test_normalize_angle ran")
		# angle = -math.pi
		# while (angle < 0):
		# 	self.assertEqual(self.circle_near_ball.normalize_angle(angle),
		# 		angle + 2 * math.pi)
		# 	angle += math.pi / 6
		# angle = 0
		# while (angle <= 2 * math.pi):
		# 	self.assertEqual(angle, 
		# 		self.circle_near_ball.normalize_angle(angle))
		# 	angle += math.pi / 6
		# angle = 3 * math.pi
		# while (angle < 4 * math.pi):
		# 	self.assertEqual(angle - 2 * math.pi,
		# 		self.circle_near_ball.normalize_angle(angle))
		# 	angle += math.pi / 6

