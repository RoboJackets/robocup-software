from dataclasses import dataclass
from typing import List, Optional
import stp.rc as rc
import stp.tactic as tactic
import stp.role as role
from rj_gameplay.skill import shoot, capture
import stp.skill as skill
import numpy as np
from math import atan2



def find_target_point(world_state:rc.WorldState) -> np.ndarray:
	goal_y = world_state.field.length_m
	cost = 0
	target_point = np.array([-0.5, goal_y])
	try_points = [np.array([-0.5, goal_y]), np.array([-0.5, goal_y]),np.array([-0.5, goal_y])]

	kicker = [robot for robot in world_state.our_robots if robot.has_ball_sense]
	if kicker:
		kicker = kicker[0]
	else:
		return None

	kicker_loc = kicker.pose[0:2]

	for point in try_points:
		angle = 3.14
		v_kick_point = point - kicker_loc
		for blocker in world_state.their_robots:
			blocker_loc = blocker.pose[0:2]
			v_kick_block = blocker_loc - kicker_loc
			point_block_ang = atan2(np.linalg.det([v_kick_block, v_kick_point]), np.dot(v_kick_block, v_kick_point))
			if point_block_ang < angle:
				angle = point_block_ang
		shoot_cost = -angle
		if shoot_cost < cost:
			target_point = point
	return target_point



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
		self.shoot = tactic.SkillEntry(shoot.Shoot(chip = False, kick_speed = 40., target_point = target_point))

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

	def tick(self, role_results: tactic.RoleResults, world_state: rc.WorldState) -> List[tactic.SkillEntry]:
		"""
		:return: list of skills
		"""
		capture_result = role_results[self.capture]
		shoot_result = role_results[self.shoot]
		if capture_result and capture_result[0].is_filled():
			return [self.capture]
		if shoot_result and shoot_result[0].is_filled():
			self.shoot.skill.target_point = find_target_point(world_state)
			return [self.shoot]
		return []

	def is_done(self, world_state) -> bool:
		return self.shoot.skill.is_done(world_state)
