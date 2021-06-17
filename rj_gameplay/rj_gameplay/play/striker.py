import stp.play as play
import stp.tactic as tactic

from rj_gameplay.tactic import striker_tactic
import stp.role as role
from stp.role.assignment.naive import NaiveRoleAssignment
import stp.rc as rc
from typing import Dict, Generic, Iterator, List, Optional, Tuple, Type, TypeVar

class Striker(play.IPlay):

	def __init__(self):
		self.role_assigner = NaiveRoleAssignment()
		self.striker_tactic = striker_tactic.StrikerTactic()

	def compute_props(self, prev_props):
		pass


	def tick(self, world_state: rc.WorldState, prev_results: role.assignment.FlatRoleResults, props)-> Tuple[Dict[Type[tactic.SkillEntry], List[role.RoleRequest]], List[tactic.SkillEntry]]:
		# Get role requests from all tactics and put them into a dictionary
		role_requests: play.RoleRequests = {}

		role_requests[self.striker_tactic] = (self.striker_tactic.get_requests(world_state, None))

		# Flatten requests and use role assigner on them
		flat_requests = play.flatten_requests(role_requests)
		flat_results = self.role_assigner.assign_roles(flat_requests, world_state, prev_results)
		role_results = play.unflatten_results(flat_results)

		skills = self.striker_tactic.tick(role_results[self.striker_tactic])
		skill_dict = {}

		skill_dict.update(role_results[self.striker_tactic])

		return (skill_dict, skills)

	def is_done(self, world_state):
		return self.striker_tactic.is_done(world_state)
		