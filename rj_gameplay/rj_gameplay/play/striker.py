import stp.play as play
import stp.tactic as tactic

from rj_gameplay.tactic import striker_tactic, assist_tactic
import stp.role as role
from stp.role.assignment.naive import NaiveRoleAssignment
import stp.rc as rc
from typing import (
    Dict,
    List,
    Tuple,
    Type,
)
import numpy as np


class Striker(play.IPlay):
    def __init__(self):
        self.striker_loc: np.ndarray = np.array([0.0, 12.0])
        self.target_point: np.ndarray = np.array([0.0, 12.0])
        self.role_assigner = NaiveRoleAssignment()
        self.assist_tactic = assist_tactic.AssistTactic(self.striker_loc)
        self.striker_tactic = striker_tactic.StrikerTactic(self.target_point)

    def compute_props(self, prev_props):
        pass

    def tick(
        self,
        world_state: rc.WorldState,
        prev_results: role.assignment.FlatRoleResults,
        props,
    ) -> Tuple[
        Dict[Type[tactic.SkillEntry], List[role.RoleRequest]],
        List[tactic.SkillEntry],
    ]:
        # Get role requests from all tactics and put them into a dictionary
        role_requests: play.RoleRequests = {}
        if not self.striker_tactic.is_done(
            world_state
        ) and not self.assist_tactic.is_done(world_state):
            role_requests[self.assist_tactic] = self.assist_tactic.get_requests(
                world_state, None
            )

        elif not self.striker_tactic.is_done(
            world_state
        ) and self.assist_tactic.is_done(world_state):
            role_requests[self.striker_tactic] = self.striker_tactic.get_requests(
                world_state, None
            )

        # Flatten requests and use role assigner on them
        flat_requests = play.flatten_requests(role_requests)
        flat_results = self.role_assigner.assign_roles(
            flat_requests, world_state, prev_results
        )
        role_results = play.unflatten_results(flat_results)

        skill_dict = {}
        if not self.striker_tactic.is_done(
            world_state
        ) and not self.assist_tactic.is_done(world_state):
            skills = self.assist_tactic.tick(
                world_state, role_results[self.assist_tactic]
            )
            skill_dict.update(role_results[self.assist_tactic])

        elif not self.striker_tactic.is_done(
            world_state
        ) and self.assist_tactic.is_done(world_state):
            skills = self.striker_tactic.tick(
                world_state, role_results[self.striker_tactic]
            )
            skill_dict.update(role_results[self.striker_tactic])
        else:
            skills = []

        return (skill_dict, skills)

    def is_done(self, world_state):
        return self.striker_tactic.is_done(world_state)
