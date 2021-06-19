from dataclasses import dataclass
from typing import List, Optional
import stp.rc as rc
import stp.tactic as tactic
import stp.role as role
from rj_gameplay.skill import shoot, capture
import stp.skill as skill
import numpy as np



# TODO: def find_target_point()
random_shoot = np.random.uniform(-0.5,0.5)

class CaptureCost(role.CostFn):
    """
    A cost function for how to choose a robot that will pass
    TODO: Implement a better cost function
    """
    def __call__(self,
                robot:rc.Robot,
                prev_result:Optional["RoleResult"],
                world_state:rc.WorldState) -> float:
        if robot.has_ball_sense:
            return 0
        else:
            robot_pos = robot.pose[0:2]
            ball_pos = world_state.ball.pos[0:2]
            dist_to_ball = np.linalg.norm(ball_pos - robot_pos)
            return dist_to_ball

class StrikerTactic(tactic.ITactic):
	"""
	A striker tactic which captures then shoots the ball
	"""

	def __init__(self, target_point: np.ndarray):
		self.target_point = target_point
		self.capture = tactic.SkillEntry(capture.Capture()) 
		self.capture_cost = CaptureCost()
		self.shoot = tactic.SkillEntry(shoot.Shoot(chip = False, kick_speed = 40., target_point = np.array([random_shoot, 12.])))

	def compute_props(self):
		pass

	def create_request(self, **kwargs) -> role.RoleRequest:

		"""
		Creates a sane default RoleRequest.
		:return: A list of size 1 of a sane default RoleRequest.
		"""
		pass

	def get_requests(self, world_state: rc.WorldState, props) -> List[tactic.RoleRequests]:

		striker_request = role.RoleRequest(role.Priority.HIGH, True, self.capture_cost)
		role_requests: tactic.RoleRequests = {}

		striker = [robot for robot in world_state.our_robots if robot.has_ball_sense]

		if striker:
			role_requests[self.capture] = []
			role_requests[self.shoot] = [striker_request]
		else:
			role_requests[self.capture] = [striker_request]
			role_requests[self.shoot] = []

		return role_requests

	def tick(self, role_results: tactic.RoleResults) -> List[tactic.SkillEntry]:
		"""
		:return: list of skills
		"""
		capture_result = role_results[self.capture]
		shoot_result = role_results[self.shoot]
		if capture_result and capture_result[0].is_filled():
			return [self.capture]
		if shoot_result and shoot_result[0].is_filled():
			return [self.shoot]
		return []

	def is_done(self, world_state) -> bool:
		return self.shoot.skill.is_done(world_state)
