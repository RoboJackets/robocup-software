import robocup
import play
import behavior
import skills.move
import skills.capture
import tactics.coordinated_pass
import constants
import main
import enum


## A demo play written during a teaching session to demonstrate play-writing
# Three robots form a triangle on the field and pass the ball A->B->C->A and so on.
class FourCornerPass(play.Play):
	class State(enum.Enum):
		## 2 robots get on the corners of a triangle,
		# while a third fetches the ball
		setup = 1

		## The robots continually pass to each other
		passing = 2

		## Once the enemy robot captures the ball
		#reset = 3

	def __init__(self):
		super().__init__(continuous=True)

		# register states - they're both substates of "running"
		self.add_state(FourCornerPass.State.setup,
					   behavior.Behavior.State.running)
		self.add_state(FourCornerPass.State.passing,
					   behavior.Behavior.State.running)

		self.add_transition(behavior.Behavior.State.start,
							FourCornerPass.State.setup, lambda: True,
							'immediately')
		self.add_transition(
			FourCornerPass.State.setup, FourCornerPass.State.passing, lambda: self.is_setup_done() , 'all subbehaviors completed')

		self.add_transition(
			FourCornerPass.State.passing, FourCornerPass.State.setup, lambda: self.subbehavior_with_name('passer').state == behavior.Behavior.
			State.completed or self.subbehavior_with_name('passer').state == behavior.Behavior.
			State.failed, 'all subbehaviors completed')

		self.square_points = [
			robocup.Point(1.5, 6),
			robocup.Point(-1.5, 6),
			robocup.Point(-1.5, 3),
			robocup.Point(1.5, 3)]

		self.direction = 1
		self.chaser_pos = robocup.Point(0,0)
		
	def execute_running(self):
		if (self.has_subbehavior_with_name('move1')) :
			self.subbehavior_with_name('move1').get_robot().set_max_speed(2.0)

		if (self.has_subbehavior_with_name('move2')) :
			self.subbehavior_with_name('move2').get_robot().set_max_speed(2.0)

		if (self.has_subbehavior_with_name('capture')):
			self.subbehavior_with_name('capture').get_robot().set_max_speed(2.0)

		if (self.has_subbehavior_with_name('chaser')):
			self.subbehavior_with_name('chaser').get_robot().set_max_speed(1.0)


		self.paint_the_field()
		#self.chaser_pos = self.chaser.robot_location

	def on_enter_setup(self):
		closestPt = min(self.square_points, key=lambda pt: pt.dist_to(main.ball().pos))
		self.otherPts = list(self.square_points)
		self.otherPts.remove(closestPt)

		farthestPt = max(self.otherPts,
						key=lambda pt: pt.dist_to(main.ball().pos))

		self.otherPts.remove(farthestPt)

		self.add_subbehavior(skills.move.Move(self.otherPts[0]), 'move1')
		self.add_subbehavior(skills.move.Move(self.otherPts[1]), 'move2')
		self.add_subbehavior(skills.capture.Capture(), 'capture')

	def on_exit_setup(self):
		#self.remove_subbehavior('move1')
		#self.remove_subbehavior('move2')
		self.remove_subbehavior('capture')

	def on_enter_passing(self):
		# pick pass from and to points
		kickFrom = min(self.square_points,
					   key=lambda pt: pt.dist_to(main.ball().pos))
		kickFromIdx = self.square_points.index(kickFrom)
		self.safer_pass()
		kickToIdx = (kickFromIdx + self.direction) % len(self.square_points)
		kickToPt = self.square_points[kickToIdx]
		if (kickToPt is self.otherPts[0]) :
			self.remove_subbehavior('move1')
			self.text = 'move2'
		else :
			self.remove_subbehavior('move2')
			self.text = 'move1'
		self.passing_machine = tactics.coordinated_pass.CoordinatedPass(kickToPt)

		self.add_subbehavior(self.passing_machine, 'passer')

		self.chaser = skills.move.Move(self.cut_of_pos(main.ball().pos))
		self.add_subbehavior(self.chaser, 'Chasing', required = True, priority = 5)

	#def execute_passing(self):
			

	def on_exit_passing(self):
		self.remove_all_subbehaviors()
		#self.remove_subbehavior(self.text)
		#self.remove_subbehavior('passing')

	def paint_the_field(self):
		for i in range(len(self.square_points)) :
			main.system_state().draw_line(robocup.Line(self.square_points[i], self.square_points[(i + 1) % 4]), (135, 0, 255), "Square")

	def safer_pass(self):
		if (True):
			self.direction = 1
		else :
			self.direction = -1

	def is_setup_done(self):
		return (self.subbehavior_with_name('capture').state == behavior.Behavior.
			State.completed)

	def cut_of_pos(self, point):
		x = point.x
		if (point.x > 1.25):
			x = 1.25
		elif (point.x < -1.25):
			x = -1.25

		y = point.y
		if (point.y > 5.75):
			y = 5.75
		elif(point.y < 3.25):
			y = 3.25
		return robocup.Point(x,y)
