import stp.play as play
import stp.tactic as tactic

from rj_gameplay.tactic import wall_tactic
import stp.skill as skill
import stp.role as role
from stp.role.assignment.naive import NaiveRoleAssignment
import stp.rc as rc
from typing import Dict, Generic, Iterator, List, Optional, Tuple, Type, TypeVar
import numpy as np


class WallBall(play.IPlay):
    """
    Test play for the wall tactic. Directs robots to form a wall between the ball and goal.
    """
    def __init__(self):
        # defaults to walling between ball pos and goal pos
        self.wall_tactic = wall_tactic.WallTactic(3)
        self.role_assigner = NaiveRoleAssignment()

    def compute_props(self, prev_props):
        pass

    def tick(
        self,
        world_state: rc.WorldState,
        prev_results: role.assignment.FlatRoleResults,
        props,
    ) -> Tuple[Dict[Type[tactic.SkillEntry], List[role.RoleRequest]], List[tactic.SkillEntry]]:
        # Get role requests from all tactics and put them into a dictionary
        role_requests: play.RoleRequests = {}
        role_requests[self.wall_tactic] = self.wall_tactic.get_requests(world_state, None)

        # Flatten requests and use role assigner on them
        flat_requests = play.flatten_requests(role_requests)
        flat_results = self.role_assigner.assign_roles(flat_requests, world_state, prev_results)
        role_results = play.unflatten_results(flat_results)

        # Get list of all skills with assigned roles from tactics
        skill_dict = {}
        skills = self.wall_tactic.tick(role_results[self.wall_tactic])
        skill_dict.update(role_results[self.wall_tactic])

        return (skill_dict, skills)

    def is_done(self ,world_state):
        return self.wall_tactic.is_done(world_state)
