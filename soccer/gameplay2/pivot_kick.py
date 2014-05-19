from enum import Enum
from behavior import *


class PivotKick(Behavior):

	class State(Enum):
		capturing = 1
		aiming = 2
		kicking = 3


	def __init__(self):
		super().__init__(continuous=False)
		self.add_state(PivotKick.State.capturing, Behavior.State.running)
		self.add_state(PivotKick.State.aiming, Behavior.State.running)
		self.add_state(PivotKick.State.kicking, Behavior.State.running)


	def execute_start(self):
		self.transition(PivotKick.State.capturing)

	def execute_capturing(self):
		self.transition(PivotKick.State.aiming)

	def execute_aiming(self):
		self.transition(PivotKick.State.kicking)

	def execute_kicking(self):
		self.transition(Behavior.State.completed)
