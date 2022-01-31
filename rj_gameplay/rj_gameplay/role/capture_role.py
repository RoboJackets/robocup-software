import stp
from rj_gameplay.skill import capture

from rj_msgs.msg import RobotIntent

class CaptureRole(stp.role.Role):
	def __init__(self, robot:stp.rc.Robot) -> None:
		super().__init__(robot)

		self.capture_skill = None
		self._state = "init"

	def set_capture(self):
		self._state = "capture"
		self.capture_skill = capture.Capture(robot=self.robot)

	def tick(self, world_state: stp.rc.WorldState) -> RobotIntent:

		self.capture_skill = capture.Capture(robot=self.robot)
		intent = self.capture_skill.tick(world_state)
		if self.capture_skill.is_done(world_state):
			self._state = "done"

		return intent

	def is_done(self, world_state) -> bool:
		return self._state == "done"