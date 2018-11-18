import main
import robocup
import behavior
import constants
import enum

import standard_play
import evaluation.ball
import tactics.coordinated_pass
import tactics.defensive_forward
import tactics.move_to_corner
import skills.move
import skills.capture


class FourCornersPass(standard_play.StandardPlay):


	class State (enum.Enum):

		collecting = 1

		moving = 2

		passing = 3


	def __init__(self):
		super().__init__(continuous=False)

		for s in FourCornersPass.State:
			self.add_state(s, behavior.Behavior.State.running)

		self.handler = None

		self.recivers = None

		self.passing_point = robocup.Point(0,0)

		self.dribble_point = robocup.Point(0,0)

		self.corners = [robocup.Point(-1.5, 3.0),robocup.Point(-1.5, 6.0), robocup.Point(1.5, 6.0), robocup.Point(1.5, 3.0)]

		self.enemies = main.their_robots()

		for opp in self.enemies:
			if opp.pos.x > -1.5 and opp.pos.x < 1.5 and opp.pos.y > 3 and opp.pos.y < 6 :
				self.hunter = opp

		self.paint_the_field()

		self.add_transition(behavior.Behavior.State.start,
                            FourCornersPass.State.collecting, lambda: True,
                            'immediately')

		self.add_transition(FourCornersPass.State.collecting, FourCornersPass.State.moving, lambda: self.subbehavior_with_name('defend').state ==
			behavior.Behavior.State.completed, 'Ball Collected')

		self.add_transition(FourCornersPass.State.moving, FourCornersPass.State.passing, lambda: self.dribbler_has_ball() and self.should_pass_from_moving(), 'Make pass')

		self.add_transition(FourCornersPass.State.passing, FourCornersPass.State.collecting, lambda: self.subbehavior_with_name('pass').state == 
			behavior.Behavior.State.cancelled or self.subbehavior_with_name('pass').state == behavior.Behavior.State.failed,'Passing: Ball Lost')

		self.add_transition(FourCornersPass.State.passing, FourCornersPass.State.moving, lambda: self.subbehavior_with_name('pass').state ==
			behavior.Behavior.State.completed,'Passed')

		self.add_transition(FourCornersPass.State.moving, FourCornersPass.State.collecting, lambda: not self.dribbler_has_ball(), 'Lost ball')


	def on_enter_collecting(self):
		self.remove_all_subbehaviors()
		# 2 man to man defenders and 1 zone defender
		self.paint_the_field()

		defensive_forward = tactics.defensive_forward.DefensiveForward()
		self.add_subbehavior(defensive_forward, 'defend', required=True)

	def on_exit_collecting(self):
		self.remove_all_subbehaviors()

	def on_enter_moving(self):
		self.holder = skills.dribble.Dribble()
		self.dribble_point = main.ball().pos
		self.paint_the_field()

		closest_corner = self.corners[0]
		self.closest_index = 0
		counter = 0
		for corner in self.corners:
			if (self.dribble_point - corner).mag() < (self.dribble_point - closest_corner).mag() :
				closest_corner = corner
				self.closest_index = counter
			counter = counter + 1

		self.holder.pos = closest_corner

		self.add_subbehavior(self.holder, 'dribble', required = True)

		if (not self.has_subbehavior_with_name('recievers')):
			self.recivers = tactics.move_to_corner.MoveToCorner()
			self.add_subbehavior(
				self.recivers, 'recivers', required=False, priority=10)

		self.recivers.pass_corner = self.closest_index

	def on_exit_moving(self):
		self.remove_subbehavior('dribble')

	def on_enter_passing(self):
		self.paint_the_field()

		self.add_subbehavior(tactics.coordinated_pass.CoordinatedPass(self.passing_point),'pass')

	def on_exit_passing(self):
		self.remove_subbehavior('pass')

	def dribbler_has_ball(self):
		return any(
			evaluation.ball.robot_has_ball(r) for r in main.our_robots())

	def should_pass_from_moving(self):
		distance_to_passer = (self.hunter.pos - self.corners[self.closest_index])

		if distance_to_passer.mag() < (self.hunter.pos - self.corners[(self.closest_index + 1) % 4]).mag() :
			self.passing_point = self.corners[(self.closest_index + 1) % 4]
			return True
		if distance_to_passer.mag() < (self.hunter.pos - self.corners[(self.closest_index - 1) % 4]).mag() :
			self.passing_point = self.corners[(self.closest_index - 1) % 4]
			return True
		return False

	def paint_the_field(self) :
		for i in range(len(self.corners)) :
			main.system_state().draw_line(robocup.Line(self.corners[i], self.corners[(i + 1) % 4]), (135, 0, 255), "Square")
