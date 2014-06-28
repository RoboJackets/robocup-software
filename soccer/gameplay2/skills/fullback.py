import single_robot_behavior
import behavior
import constants
import robocup
from enum import Enum

class Fullback(single_robot_behavior.SingleRobotBehavior):

	class Role(Enum):
		Marking = 1
		AreaMarking = 2
		MultiMark = 4
		Intercept = 8
		Support = 16
		Receiving = 32
		Passing = 64

	class Side(Enum):
		Left = 1
		Center = 2
		Right = 3

	def __init__(self, side, role):
		super().__init__(continuous=True)
		self._block_robot = None
		self._side = side
		self._roles = role

		self.add_state()


	@property 
	def block_robot(self):
		return self._block_robot
	@block_robot.setter
	def block_robot(self, value):
		self._block_robot = value

	@property 
	def side(self):
		return self._side
	@side.setter
	def side(self, value):
		self._side = value

	@property 
	def roles(self):
		return self._roles
	@role.setter
	def roles(self, value):
		self._roles = value