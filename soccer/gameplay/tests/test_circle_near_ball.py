import unittest
import main
import robocup
import math
import tactics.stopped.circle_near_ball
import constants

class Moc_Ball:
	def __init__(self, x, y):
		self.pos = robocup.Point(x, y)

class TestCircleNearBall(unittest.TestCase):
	def __init__(self, *args, **kwargs):
		super(TestCircleNearBall, self).__init__(*args, **kwargs)

	def setUp(self):
		self.circle_near_ball = tactics.stopped.circle_near_ball.CircleNearBall()

	def test_get_circle_points(self):
		width = constants.Field.Width
		length = constants.Field.Length
		
		# Checks that the points around the ball are within the bounds of the field
		def assert_circle_points(self):
			array = self.circle_near_ball.get_circle_points(6)
			for point in array:		
				self.assertLessEqual(point.x, width * 0.5, "X position of points are outside the field.")
				self.assertGreaterEqual(point.x, width * -0.5, "X position of points are outside the field.")
				
				self.assertLessEqual(point.y, length, "Y positions of points are outside the field.")
				self.assertGreaterEqual(point.y, 0, "Y positions of points are outside the field.")
		
		# Checks a point on the field to see if the circle around it is valid
		def test_point(self, x, y):
			main.set_ball(Moc_Ball(x, y))
			assert_circle_points(self)

		# Checks center of field
		test_point(self, constants.Field.CenterPoint.x, constants.Field.CenterPoint.y)
		
		# Checks a point that should be clearly outside the field
		test_point(self, width * -1, length * -1)
		
		# Checks bottom left corner of field
		test_point(self, width * -1 / 2, 0)

		# Checks (0, 0). Should be right in front of our goal
		

		# Checks bottom right corner of field
		test_point(self, width / 2, 0)

		# Checks middle right part of field
		test_point(self, width / 2, length / 2)

		# Checks a point just inside the field
		test_point(self, width / 2 - constants.Robot.Radius, constants.Robot.Radius)
		
		# Checks a point just outside of the field
		test_point(self, width / 2 + constants.Robot.Radius, constants.Robot.Radius * -1)

	def test_normalize_angle(self):
		angle = -math.pi
		while (angle < 0):
			self.assertAlmostEqual(self.circle_near_ball.normalize_angle(angle),
				angle + 2 * math.pi, msg="Unexpected value for normalize angle.")
			angle += math.pi / 6
		angle = 0
		while (angle <= 2 * math.pi):
			self.assertAlmostEqual(angle, 
				self.circle_near_ball.normalize_angle(angle), 
				msg="Unexpected value for normalize angle.")
			angle += math.pi / 6
		angle = 3 * math.pi
		while (angle < 4 * math.pi):
			self.assertAlmostEqual(angle - 2 * math.pi,
				self.circle_near_ball.normalize_angle(angle), 
				msg="Unexpected value for normalize angle.")
			angle += math.pi / 6
