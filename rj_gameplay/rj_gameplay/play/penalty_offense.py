import stp.play as play
import stp.tactic as tactic

from rj_gameplay.tactic import (
    striker_tactic,
    nmark_tactic,
    goalie_tactic,
    pass_seek,
    wall_tactic,
    move_tactic,
)
import stp.skill as skill
import stp.role as role
from stp.role.assignment.naive import NaiveRoleAssignment
import stp.rc as rc
from typing import Dict, Generic, Iterator, List, Optional, Tuple, Type, TypeVar, Any
import numpy as np


class PenaltyOffense(play.IPlay):
    """Move all robots to our half, but away from ball to prep for penalty kick"""

    def __init__(self, action_client_dict: Dict[Type[Any], List[Any]]):
        self.tactics = [
            goalie_tactic.GoalieTactic(True),
            striker_tactic.LineKickStrikerTactic(action_client_dict, None),
            move_tactic.Move((1.8, 0.0), priority=role.Priority.LOW),
            move_tactic.Move((2.1, 0.0), priority=role.Priority.LOW),
            move_tactic.Move((2.4, 0.0), priority=role.Priority.LOW),
            move_tactic.Move((2.7, 0.0), priority=role.Priority.LOW),
        ]
        self.role_assigner = NaiveRoleAssignment()

    def compute_props(self, prev_props):
        pass

    def tick(
        self,
        world_state: rc.WorldState,
        prev_results: role.assignment.FlatRoleResults,
        props,
    ) -> Tuple[
        Dict[Type[tactic.SkillEntry], List[role.RoleRequest]], List[tactic.SkillEntry]
    ]:
        # Get role requests from all tactics and put them into a dictionary
        role_requests: play.RoleRequests = {
            tactic: tactic.get_requests(world_state, None) for tactic in self.tactics
        }
        # Flatten requests and use role assigner on them
        flat_requests = play.flatten_requests(role_requests)
        flat_results = self.role_assigner.assign_roles(
            flat_requests, world_state, prev_results
        )
        role_results = play.unflatten_results(flat_results)
        # Get list of all SkillEntries from all tactics
        skills = []
        tac_index = 0
        for tactic in self.tactics:
            if tac_index == 1:  # TODO BAD HACK
                skills += tactic.tick(world_state, role_results[tactic])
            else:
                skills += tactic.tick(world_state, role_results[tactic])
            tac_index += 1
        # Get all role assignments
        # SkillEntry to (list of?) RoleResult
        skill_dict = {}
        for tactic in self.tactics:
            skill_dict.update(role_results[tactic])
        return (skill_dict, skills)

    def is_done(self, world_state):
        return self.tactics[-1].is_done(world_state)
