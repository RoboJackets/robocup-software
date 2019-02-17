import robocup
import play
import behavior
import skills.move
import skills.capture
import tactics.coordinated_pass
import constants
import main
import enum


## A testing play to demonstrate our ability to pass and recieve balls
# One robot will pursue the ball while three other robots will pass the ball amongst themselves
# they can only pass the ball at the corners and recieve at the corners and they cannot move
# through the center of the square
class FourCornerPass(play.Play):
	class State(enum.Enum):
		## One robot goes captures the ball while the other two gets on corners
		# pursuing robot does not move on the first instance
		setup = 1

		## The robots pass to each other and begin setting up the next pass
		# while the chasing robot goes for the ball.
		passing = 2

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
		#transition when ball is captured
		self.add_transition(
			FourCornerPass.State.setup, FourCornerPass.State.passing, lambda: self.is_setup_done() , 'all subbehaviors completed')

		#transition when ball is passed or failed to pass
		self.add_transition(
			FourCornerPass.State.passing, FourCornerPass.State.setup, lambda: self.subbehavior_with_name('passer').state == behavior.Behavior.
			State.completed or self.subbehavior_with_name('passer').state == behavior.Behavior.
			State.failed, 'all subbehaviors completed')

		#change this to adjust the square size
		variable_square = 4.5

		#NEVER CHANGE THIS
		length = constants.Field.Length
		width = constants.Field.Width

		#if the square size is smaller than the width shrink it down otherwise
		#max value is the width of the field.
		if (variable_square < width) :
			width = variable_square

		#radius of robot
		self.bot = constants.Robot.Radius
		# the largest and smallest x and y postion possible for the square
		self.max_x = (width / 2 - self.bot * 3)
		self.min_x = -self.max_x
		self.max_y = length/2 + self.max_x
		self.min_y = length/2 - self.max_x

		# the four courners
		self.square_points = [
			robocup.Point(self.max_x, self.max_y),
			robocup.Point(self.min_x, self.max_y),
			robocup.Point(self.min_x, self.min_y),
			robocup.Point(self.max_x, self.min_y)]

		# speed of the hunting robots
		self.normal_speed = 2.0
		# speed of the defending robots can decrease value to make it easier for offense
		self.defense_speed = variable_square/3.0

		# picks the direction to pass to. TODO make actual smart pass selection
		self.direction = 1

	# constanly running changes and updates
	def execute_running(self):
		# checks if there is an offensive behavior for the various plays and set the normal speed
		if (self.has_subbehavior_with_name('move1')) :
			robo = self.subbehavior_with_name('move1').robot
			if (robo != None) :
				robo.set_max_speed(self.normal_speed)

		if (self.has_subbehavior_with_name('move2')) :
			robo = self.subbehavior_with_name('move2').robot
			if (robo != None) :
				robo.set_max_speed(self.normal_speed)

		if (self.has_subbehavior_with_name('capture')):
			robo = self.subbehavior_with_name('capture').robot
			if (robo != None) :
				robo.set_max_speed(self.normal_speed)

		if (self.has_subbehavior_with_name('premove')) :
			robo = self.subbehavior_with_name('premove').robot
			if (robo != None) :
				robo.set_max_speed(self.normal_speed)

		# checks if there is a chasing robot and set their speed to defense speed
		if (self.has_subbehavior_with_name('Chasing')):
			robo = self.subbehavior_with_name('Chasing').robot
			if (robo != None) :
				robo.set_max_speed(self.defense_speed)

		# keep the iso point stuck in the corner
		if (self.has_subbehavior_with_name('Iso')) :
			self.iso.pos = robocup.Point(-constants.Field.Width/2, 0)

		# chasing robot positon should always follow the ball within a smaller inside box of the
		# four corners.
		if (self.has_subbehavior_with_name('Chasing')) :
			self.chaser.pos = self.cut_of_pos(main.ball().pos)

		#draw the four courner field
		self.paint_the_field()

	def on_enter_setup(self):
		# find where we will consider the ball is closest too and passing from
		closestPt = min(self.square_points, key=lambda pt: pt.dist_to(main.ball().pos))

		#points that the other robots can move to
		self.otherPts = list(self.square_points)
		self.otherPts.remove(closestPt)

		# remove the point that's diagonal of the closest point
		farthestPt = max(self.otherPts,
						key=lambda pt: pt.dist_to(main.ball().pos))

		self.otherPts.remove(farthestPt)


		# move the other two robots to the other point locations
		self.add_subbehavior(skills.move.Move(self.otherPts[0]), 'move1', required = False)
		self.add_subbehavior(skills.move.Move(self.otherPts[1]), 'move2', required = False)
		# send the closest robot to capture the ball
		self.add_subbehavior(skills.capture.Capture(), 'capture')

	def on_exit_setup(self):
		self.remove_subbehavior('capture')

	def on_enter_passing(self):
		# get the point passing from
		kickFrom = min(self.square_points,
					   key=lambda pt: pt.dist_to(main.ball().pos))
		# get the index of where you're passing from
		kickFromIdx = self.square_points.index(kickFrom)
		# decide which direction you're passing to
		self.safer_pass()
		# based on which direction you're passing to pick that index
		kickToIdx = (kickFromIdx + self.direction) % len(self.square_points)
		# get that point
		kickToPt = self.square_points[kickToIdx]

		# remove the subbehavior that's not recieving the pass
		if (kickToPt is self.otherPts[0]) :
			self.remove_subbehavior('move1')
			self.text = 'move2'
		else :
			self.remove_subbehavior('move2')
			self.text = 'move1'

		# while pass is preparing get ready for the next pass
		premove = skills.move.Move(self.square_points[(kickToIdx + self.direction) % len(self.square_points)])
		# get ready to pass the ball
		self.passing_machine = tactics.coordinated_pass.CoordinatedPass(kickToPt)

		#if (self.has_subbehavior_with_name('Chasing')) :
		#	self.remove_subbehavior('Chasing')

		self.add_subbehavior(self.passing_machine, 'passer', required = True, priority = 10)
		self.add_subbehavior(premove, 'premove', required = True)


		# add a robot to chase after the ball.
		if (not self.has_subbehavior_with_name('Chasing')):
			self.chaser = skills.move.Move(self.cut_of_pos(main.ball().pos))
			self.add_subbehavior(self.chaser, 'Chasing', required = True, priority = 2)

		# if we have too many robots isolate one of the robots so they don't help in the play
		if (not self.has_subbehavior_with_name('Iso') and len(main.our_robots()) == 6) :
			self.iso = skills.move.Move(robocup.Point(-constants.Field.Width/2, 0))
			self.add_subbehavior(self.iso,'Iso', required = True, priority = 1)
			
	# remove subbehaviors 
	def on_exit_passing(self):
		self.remove_subbehavior(self.text)
		self.remove_subbehavior('passer')
		self.remove_subbehavior('Chasing')
		self.remove_subbehavior('premove')

	# takes in the square points and form lines and create a square on the field
	def paint_the_field(self):
		for i in range(len(self.square_points)) :
			main.system_state().draw_line(robocup.Line(self.square_points[i], self.square_points[(i + 1) % 4]), (135, 0, 255), "Square")

	#TODO decide which the better direction to pass is
	def safer_pass(self):
		if (True):
			self.direction = 1
		else :
			self.direction = -1

	# checks if capture is completed and setup is finish
	def is_setup_done(self):
		return (self.subbehavior_with_name('capture').state == behavior.Behavior.
			State.completed)

	# if a point is outside of a smaller box that is the square points - 3 robot radius'
	# return the closest point inside that box.
	def cut_of_pos(self, point):
		x = point.x
		if (point.x > (self.max_x - 3 * self.bot)):
			x = self.max_x - 3 * self.bot
		elif (point.x < (self.min_x + 3 *self.bot)):
			x = self.min_x + 3 *self.bot

		y = point.y
		if (point.y > (self.max_y - 3 * self.bot)):
			y = self.max_y - 3 * self.bot
		elif(point.y < (self.min_y + 3 * self.bot)):
			y = self.min_y + 3 * self.bot
		return robocup.Point(x,y)
