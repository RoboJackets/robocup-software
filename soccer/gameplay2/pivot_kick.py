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

		self.add_transition(Behavior.State.start, PivotKick.State.capturing, lambda: True)
		self.add_transition(PivotKick.State.capturing, PivotKick.State.aiming, lambda: True)
		self.add_transition(PivotKick.State.aiming, PivotKick.State.kicking, lambda: True)
		self.add_transition(PivotKick.State.kicking, Behavior.State.completed, lambda: True)


	def execute_capturing(self):
		pass

	def execute_aiming(self):
		pass

	def execute_kicking(self):
		pass
