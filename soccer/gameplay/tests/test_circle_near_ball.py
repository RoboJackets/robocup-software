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
		width = constants.Field.Width
		length = constants.Field.Length
		print("Field Width: " + str(width))
		print("Field Length: " + str(length))
		def assert_circle_points(self):
			array = self.circle_near_ball.get_circle_points(6)
			for point in array:		
				self.assertTrue(point.x >= width * -1 / 2 and point.x <= width / 2, msg="failure")
				self.assertTrue(point.y >= 0 and point.y <= length, msg="failure")
		
		def test_point(self, x, y):
			main.set_ball(Moc_Ball(x, y))
			assert_circle_points(self)

		test_point(self, constants.Field.CenterPoint.x, constants.Field.CenterPoint.y)
		test_point(self, -100, -100)
		test_point(self, width * -1, 0)
		test_point(self, 0, 0)
		test_point(self, width, 0)
		test_point(self, width, length / 2)
		test_point(self, width - constants.Robot.Radius, constants.Robot.Radius)
		test_point(self, width + constants.Robot.Radius, constants.Robot.Radius * -1)

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

