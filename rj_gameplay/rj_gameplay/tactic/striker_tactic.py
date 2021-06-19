from dataclasses import dataclass
from typing import List, Optional
import stp.rc as rc
import stp.tactic as tactic
import stp.role as role
from rj_gameplay.skill import shoot, capture, receive, pivot_kick
import stp.skill as skill
import numpy as np
from math import atan2

def find_striker(robot:rc.Robot, world_state: rc.WorldState):
	cost = 0
	goal_loc = world_state.field.their_goal_loc
	kicker = world_state.our_robots[robot.id]
	left_end = np.array([-0.5, world_state.field.length_m])
	right_end = np.array([0.5, world_state.field.length_m])
	dist_to_goal = np.linalg.norm(goal_loc - kicker.pose[0:2])

	if dist_to_goal > 5:
		return 9999
	else:
		u_vec_kicker_left = (left_end - kicker.pose[0:2]) / np.linalg.norm(left_end - kicker.pose[0:2])
		u_vec_kicker_right = (right_end - kicker.pose[0:2]) / np.linalg.norm(right_end - kicker.pose[0:2])
		shoot_range = atan2(np.linalg.det([u_vec_kicker_left, u_vec_kicker_right]), np.dot(u_vec_kicker_left, u_vec_kicker_right))
		#TODO find weight
		cost -= shoot_range
		for opp_robot in world_state.their_robots:
			u_vec_kicker_opp = (opp_robot.pose[0:2] - kicker.pose[0:2]) / np.linalg.norm(opp_robot.pose[0:2] - kicker.pose[0:2])

			if np.dot(u_vec_kicker_left, u_vec_kicker_right) < np.dot(u_vec_kicker_left, u_vec_kicker_opp) and \
				np.dot(u_vec_kicker_left, u_vec_kicker_right) < np.dot(u_vec_kicker_right, u_vec_kicker_opp):
				cost += (world_state.field.length_m - opp_robot.pose[1]) / (world_state.field.length_m - kicker.pose[1]) 
		return cost


# def build_cost_list(world_state: rc.WorldState):
# 	return [find_striker(robot, world_state) for robot in world_state.our_robots]


class StrikerCost(role.CostFn):
    def __call__(self, robot: rc.Robot, prev_result: Optional["RoleResult"], world_state: rc.WorldState) -> float:

    	return find_striker(robot, world_state)

class CaptureCost(role.CostFn):
    def __call__(self, robot: rc.Robot, prev_result: Optional["RoleResult"], world_state: rc.WorldState) -> float:

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

	def __init__(self, pass_target_point: np.ndarray, shoot_target_point: np.ndarray):
		# self.target_point = target_point
		self.receive = tactic.SkillEntry(receive.Receive()) 
		# self.capture = capture.SkillEntry(capture.Capture()) 
		self.pivot_kick = tactic.SkillEntry(pivot_kick.PivotKick(target_point = pass_target_point, chip=False, kick_speed=4.0)) 
		self.shoot = tactic.SkillEntry(shoot.Shoot(chip = False, kick_speed = 30., target_point = shoot_target_point))
		self.striker_cost = StrikerCost()
		self.capture_cost = CaptureCost()
		

	def compute_props(self):
		pass

	def create_request(self, **kwargs) -> role.RoleRequest:

		"""
		Creates a sane default RoleRequest.
		:return: A list of size 1 of a sane default RoleRequest.
		"""
		pass

	def get_requests(self, world_state: rc.WorldState, props) -> List[tactic.RoleRequests]:
		"""
		return: a list of size 3 of role requests
		"""

		role_requests: tactic.RoleRequests = {}

		striker_request = role.RoleRequest(role.Priority.HIGH, True, self.striker_cost)
		capture_pass_request = role.RoleRequest(role.Priority.HIGH, True, self.capture_cost)

		cost_list = [find_striker(robot, world_state) for robot in world_state.our_robots]
		striker_id = cost_list.index(min(cost_list))

		# striker = [robot for robot in world_state.our_robots if robot.has_ball_sense]
		if world_state.our_robots[striker_id].has_ball_sense:
			role_requests[self.pivot_kick] = []
			role_requests[self.receive] = []
			role_requests[self.shoot] = [striker_request]

		else:
			role_requests[self.pivot_kick] = [capture_pass_request]
			role_requests[self.receive] = [striker_request]
			role_requests[self.shoot] = []

		# if striker:
		# 	role_requests[self.receive] = []
		# 	role_requests[self.shoot] = [striker_request]
		# else:
		# 	role_requests[self.receive] = [striker_request]
		# 	role_requests[self.shoot] = []

		return role_requests

	def tick(self, role_results: tactic.RoleResults, world_state: rc.WorldState) -> List[tactic.SkillEntry]:
		"""
		:return: list of skills
		"""
		pass_result = role_results[self.pivot_kick]
		receive_result = role_results[self.receive]
		shoot_result = role_results[self.shoot]

		if pass_result and pass_result[0].is_filled() and receive_result and receive_result[0].is_filled() and shoot_result and shoot_result[0].is_filled():
			#TODO add target for shoot
			if self.pivot_kick.skill.pivot.is_done(world.state) and self.receive.skill.capture.is_done(world_state):
				return [self.pivot_kick, self.receive, self.shoot]
			elif pass_result and pass_result[0].is_filled() and receive_result and receive_result[0].is_filled():
				self.pivot_kick.skill.target_point = np.array(receive_result[0].role.robot.pose[0:2])
				if self.pivot_kick.skill.pivot.is_done(world_state):
					return [self.pivot_kick, self.receive]
				else:
					return [self.pivot_kick]

		elif pass_result and pass_result[0].is_filled():
			return [self.pivot_kick]

		return []

	def is_done(self, world_state) -> bool:
		return self.shoot.skill.is_done(world_state)
