import main
import robocup
import behavior
import constants
import enum

import standard_play
import evaluation
import situational_play_selection
import tactics.coordinated_pass
import skills.move
import skills.capture
import random

## Basic Offensive Pileup play
# Has one robot capture the ball
# One robot goes to a dropback point behind the pileup
# One robot goes to the side of the pileup

class BasicOffensivePileup(standard_play.StandardPlay):

	_situation_list = [
		situational_play_selection.SituationalPlaySelector.Situation.OFFENSIVE_PILEUP
		]

	def __init__ (self):
		super().__init__(continuous=False)

		self.add_transition(behavior.Behavior.State.start,
                            behavior.Behavior.State.running, lambda: True,
                            'Immediately')

	def on_enter_running(self):
		ball = main.ball().pos 
		dropback_point = robocup.Point(ball.x, 2*ball.y/3) #Since we have to divide by ball.x, if its zero, make it slightly positive
		if (ball.x == 0):
			ball.x += 0.01
		side_point = robocup.Point((-(abs(ball.x)/ball.x) * 0.4 *constants.Field.Width)+ball.x, ball.y)
		self.add_subbehavior(skills.move.Move(dropback_point), 'move to drop back')
		self.add_subbehavior(skills.move.Move(side_point), 'move to side point')
		self.add_subbehavior(skills.capture.Capture(), 'capture ball')