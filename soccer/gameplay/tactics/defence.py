import composite_behavior
import behavior
import constants
import robocup
import evaluation.window_evaluator
import main
from enum import Enum
import math
import tactics.positions.goalie

class Defence(composite_behavior.CompositeBehavior):


	def __init__(self):
		super().__init__(continuous=True)

		self.add_transition(behavior.Behavior.State.start,
			behavior.Behavior.State.running,
			lambda:True,
			"immediately")

		g = tactics.positions.goalie.Goalie()
		self.add_subbehavior(g, "Goalie", required=True)


	def execute_running(self):
		pass