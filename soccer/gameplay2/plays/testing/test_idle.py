import play
import tactics.idle
import robocup

class TestIdle(play.Play):

	def __init__(self):
		super().__init__(continuous=True)
		print("init 2\n")
		self.add_transition(behavior.Behavior.State.start,
			behavior.Behavior.State.running,
			lambda: True,
			"immediately")
		print("init test idle\n")
		c = 0
		for robot in vector_OurRobot:
			print(str(c))
			i = idle()
			i.robots.append(robot)
			self.add_subbehavior(i, name="robot"+str(c), required=False, priority=len(vector_OurRobot-c))
			c += 1
