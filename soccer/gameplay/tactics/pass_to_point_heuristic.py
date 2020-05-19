import constants
import robocup
import time

class PassToPointHeuristic():

	def __init__(self, x ,y):
		self.x = x
		self.y = y

	def calc_pass_point(self):
		return (robocup.Point(self.x, self.y))
