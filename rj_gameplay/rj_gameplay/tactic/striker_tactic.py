from dataclasses import dataclass
from typing import List, Optional
import stp.rc as rc
import stp.tactic as tactic
import stp.role as role
from rj_gameplay.skill import shoot, capture #receive
import stp.skill as skill
import numpy as np


class shoot_cost(role.CostFn):
    def __call__(self, robot: rc.Robot, prev_result: Optional["RoleResult"], world_state: rc.WorldState) -> float:

    	#TODO: elaborate cost function:
    	if robot.id == 1:
    		return 0
    	else:
    		return 999

    	# if -world_state.field.penalty_long_dist_m / 2 - 0.5 < robot.pose[0] < \
    	# 	world_state.field.penalty_long_dist_m / 2 + 0.5 and \
    	# 	world_state.field.their_goal_loc - world_state.field.penalty_short_dist_m - 0.5 < \
    	# 	robot.pose[1] < world_state.field.their_goal_loc[1] - 0.2:
    	# 	return 0
    	# else:
    	# 	return 999


class StrikerTactic(tactic.ITactic):
	"""
	A striker tactic which captures then shoots the ball
	"""

	def __init__(self):
		self.capture = tactic.SkillEntry(capture.Capture()) 
		#TODO: add receive skill 
		self.cost = shoot_cost()
		self.shoot = tactic.SkillEntry(shoot.Shoot(chip = False, kick_speed = 40.))

	def compute_props(self):
		pass

	def create_request(self, **kwargs) -> role.RoleRequest:

		"""
		Creates a sane default RoleRequest.
		:return: A list of size 1 of a sane default RoleRequest.
		"""
		pass

	def get_requests(self, world_state: rc.WorldState, props) -> List[tactic.RoleRequests]:

		striker_request = role.RoleRequest(role.Priority.HIGH, True, self.cost)
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
