import stp
from rj_gameplay.skill import mark

from rj_msgs.msg import RobotIntent

class MarkerRole(stp.role.Role):
	def __init__(self, robot:stp.rc.Robot) -> None:
		super().__init__(robot)

		self.mark_skill = None
		# self._state = "init"
		#delete if not needed
		

	def tick(self, world_state: stp.rc.WorldState) -> RobotIntent:

		self.mark_skill = mark.Mark(robot=self.robot)
		intent = self.mark_skill.tick(world_state)
		
			# if self.mark_skill.is_done(world_state):
			# 	self._state = "keep_marking"
		# if opp.has_ball_sense for opp in world_state.their_robots:

		return intent

	def is_done(self, world_state) -> bool:
		return self._state == "done"


