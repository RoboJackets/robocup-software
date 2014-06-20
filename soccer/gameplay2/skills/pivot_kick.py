from enum import Enum
import behavior


class PivotKick(behavior.Behavior):

	class State(Enum):
		capturing = 1
		aiming = 2
		kicking = 3


	def __init__(self):
		super().__init__(continuous=False)
		self.add_state(PivotKick.State.capturing, behavior.Behavior.State.running)
		self.add_state(PivotKick.State.aiming, behavior.Behavior.State.running)
		self.add_state(PivotKick.State.kicking, behavior.Behavior.State.running)

		self.add_transition(behavior.Behavior.State.start, PivotKick.State.capturing, lambda: True, 'immediate')
		self.add_transition(PivotKick.State.capturing, PivotKick.State.aiming, lambda: True, 'has ball')
		self.add_transition(PivotKick.State.aiming, PivotKick.State.kicking, lambda: True, 'aim error < threshold')
		self.add_transition(PivotKick.State.kicking, behavior.Behavior.State.completed, lambda: True, 'kick complete')

		self.add_transition(PivotKick.State.aiming, PivotKick.State.capturing, lambda: False, 'fumble')
		self.add_transition(PivotKick.State.kicking, PivotKick.State.capturing, lambda: False, 'fumble')


	def execute_capturing(self):
		pass

	def execute_aiming(self):
		pass

	def execute_kicking(self):
		pass
