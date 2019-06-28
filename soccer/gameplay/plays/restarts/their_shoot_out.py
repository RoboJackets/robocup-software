import standard_play
import behavior
import main
import robocup
import play
import enum
import skills.move
import constants
import evaluation

class TheirShootOut(play.Play):
	class State(enum.Enum):
		#where the goalie should start before the game begins
		starting = 0
		#prevent the shooting robot from chipping of the robot and shooting a straight shot
		block = 1
		#go recieve the ball
		capture = 2

	def __init__(self):
		super().__init__(continuous=True)

		for state in TheirShootOut.State :
			self.add_state(state, behavior.Behavior.State.running)

		self.add_transition(behavior.Behavior.State.start,
							TheirShootOut.State.starting, lambda: True,
							'immediately')

		self.add_transition(TheirShootOut.State.starting, 
							TheirShootOut.State.block, 
							lambda: main.game_state().is_playing(),
							'block')

		#go capture if enemy robot doesn't have the ball or is within chip range
		self.add_transition(TheirShootOut.State.block,
							TheirShootOut.State.capture, lambda: not self.has_ball() or self.in_chip_distance(),
							'capture')

		#go back to blocking if the robot reclaims control of the ball and is outside chip range.
		self.add_transition(TheirShootOut.State.capture,
							TheirShootOut.State.block, lambda: self.has_ball() and not self.in_chip_distance(),
							'reblock')

		self.block_percentage = .50
		self.chip_distance = 3.0
		self.ball_lost_distance = 0.5

	def on_enter_starting(self):
		start_point = constants.Field.OurGoalSegment.center()
		self.add_subbehavior(skills.move.Move(start_point), 'start')

	def on_exit_starting(self):
		self.remove_all_subbehaviors()

	def on_enter_block(self):
		#find ideal point to move too and move there
		move_to_point = self.get_transition_point()
		self.blocker = skills.move.Move(move_to_point)
		self.add_subbehavior(self.blocker, 'block')

	def execute_block(self): 
		#update point
		self.blocker.pos = self.get_transition_point()
		
	def on_exit_block(self):
		self.remove_all_subbehaviors()

	def on_enter_capture(self):
		#capture ball
		self.add_subbehavior(skills.capture.Capture(),'capture')

	def on_exit_capture(self):
		self.remove_all_subbehaviors()

	@classmethod
	def score (cls):
		gs = main.game_state()
		return 0 if gs.is_penalty_shootout() and gs.is_their_penalty() else float("inf")

	@classmethod
	def is_restart (cls):
		return False

	@classmethod
	def handles_goalie(cls):
		return True

	# test all robots in their fleet to see if one has the ball
	def has_ball(self):
		closeIndicator = False
		for r in main.their_robots():
			closeIndicator = closeIndicator or robocup.Segment(r.pos, main.ball().pos).length() < self.ball_lost_distance
		return closeIndicator

	#test all robots in their fleet to see if one is within chip distance of the goal
	def in_chip_distance(self):
		chipIndicator = False
		for r in main.their_robots():
			chipIndicator = chipIndicator or robocup.Segment(r.pos, constants.Field.OurGoalSegment.nearest_point(r.pos)).length() < self.chip_distance

		return chipIndicator

	#find the ideal point to move the blocking 
	def get_transition_point(self):
		segment = constants.Field.OurGoalSegment
		line = robocup.Line(main.ball().pos, main.their_robots()[0].pos)

		#find the point that the robot is most likely going to shoot at
		goal_intercept = segment.line_intersection(line)

		#if robot is not lined up with the goal assume the bot will shoot center
		if goal_intercept == None :
			goal_intercept = segment.center()

		#get the vector of the ball to the ideal shot point
		ball_to_goal_intercept = goal_intercept - main.ball().pos
		
		#normalize the vector
		ball_to_goal_intercept = ball_to_goal_intercept.normalized()

		#the ideal spot to be will be the chip distance times the normalized vector from the ball's
		#current point.
		ideal_defence = ball_to_goal_intercept * self.chip_distance + main.ball().pos

		#find the line of the shot the robot will make
		ball_to_goal_segment = robocup.Segment(main.ball().pos, goal_intercept)

		#this is the point to get our robot to block the shot the quickest.
		fastest_defence = ball_to_goal_segment.nearest_point(main.our_robots()[0].pos)

		#if robot is blocking the shot already just go to the ideal point otherwise average the vectors
		#based on the blocking percentage.
		if (main.our_robots()[0].pos.dist_to(fastest_defence) < (constants.Robot.Radius /4)):
			move_to_point = ideal_defence
		else:
			move_to_point = robocup.Point((ideal_defence.x * (self.block_percentage)) + (fastest_defence.x * (1 - self.block_percentage)), 
			((ideal_defence.y * self.block_percentage) + fastest_defence.y * (1 - self.block_percentage)))

		return move_to_point

