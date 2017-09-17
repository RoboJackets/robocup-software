import unittest
import main
import robocup
import evaluation.field
import constants

class Moc_Ball:
	def __init__(self, x, y):
		self.pos = robocup.Point(x, y)

class TestBall(unittest.TestCase):
	def __init__(self, *args, **kwargs):
		super(TestBall, self).__init__(*args, **kwargs)

	def test_is_moving_towards_our_goal(self):
		print("Hello World")

	def test_is_in_our_goalie_zone(self):
		print("Hello World")