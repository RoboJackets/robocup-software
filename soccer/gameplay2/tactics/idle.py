import behavior
import constants
import math

class Idle(behavior.Behavior):

	def __init__(self):
		super().__init__(continuos=True)
		self.robots = {}
		self.add_transition(behavior.Behavior.State.start, behavior.Behavior.State.running, lambda: True, "immediately")

	def execute_running(self):
		pass
		if not main.ball().valid or len(robots) == 0:
			return

		radius = constants.Field.CenterRadius + constants.Robot.radius + 0.01

		perRobot = (2 * constants.Robot.radius * 1.25) / radius * (180.0 / math.pi)

		dir = (Point(0,0) - main.ball().pos).normalized() * radius
		dir.rotate(Point(0,0), -perRobot * ())

		for robot in robots:
			if True: # robot.isVisible
				robot.move_to(main.ball().pos + dir)
				robot.face(main.ball().pos)

				# Avoid the ball even on our restart
				robot.set_avoid_ball_radius(constants.Field.CenterRadius)
				# Don't leave gaps
				robot.avoid_all_teammates(True)
			dir.rotate(Point(0,0), perRobot)
