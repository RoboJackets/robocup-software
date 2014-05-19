
from enum import Enum


# Things to consider:
# * goal-oriented vs continuous behaviors
# * interruptible yes/no?
# * get/set robot.  role?  return a MotionConstraints object?  accept a MotionConstraints object?
# states and sub-states


# Behaviors are state machines and these are the available states
class State(Enum):
	start = 1
	running = 2

	# note: these two states are only relevant for non-continuous behaviors
	succeeded = 3
	failed = 4



class Behavior:

	def __init__(self, continuous):
		pass


	def run(self):
		pass




	def transition(self):
		pass





	@property
	def state(self):
	    return self._state
	@state.setter
	def state(self, value):
	    self._state = value


	# continuous: a behavior that accomplishes a specific task, then completes (example: shooting at the goal)
	# noncontinuous: a behavior that continually runs until told to stop (example: zone defense)
	@property
	def is_continuous(self):
	    return self._is_continuous
	


