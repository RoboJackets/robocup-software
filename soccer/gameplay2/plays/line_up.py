from play import *
from behavior import *
from skills.move import Move


class LineUp(Play):

	def __init__(self):
		super().__init__()

		self.add_transition(Behavior.State.start, Behavior.State.running, lambda: True, 'immediately')
		self.add_transition(Behavior.State.running, Behavior.State.completed,
			lambda:
				all([b.behavior_state == Behavior.State.completed for b in self.subbehaviors]),
			'all robots reach target positions'
			)


	@Play.robots.setter
	def robots(self, robots):
		super().robots(robots)

		# build a list of Points for where the robots should go
		start_x = -1.0
		start_y = 0.5
		spacing_y = 0.25
		points = [Point(start_x, start_y + i * spacing_y) for i in range(0, len(robots))]

		self.subbehaviors = [Move(pt) for pt in points]

		# FIXME: assign behaviors more smartly
		for i in range(0, len(robots)):
			self.subbehaviors[i].robot = self.robots[i]


	def execute_running(self):
		for subbehavior in self.subbehaviors:
			subbehavior.run()


	# a list of Move behaviors
	@property
	def subbehaviors(self):
	    return self._subbehaviors
	@subbehaviors.setter
	def subbehaviors(self, value):
	    self._subbehaviors = value
