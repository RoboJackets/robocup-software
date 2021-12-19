import stp.play as play
import stp.tactic as tactic

from rj_gameplay.tactic import wall_tactic
import stp.skill as skill
import stp.role as role
from stp.role.assignment.naive import NaiveRoleAssignment
import stp.rc as rc
from typing import Dict, Generic, Iterator, List, Optional, Tuple, Type, TypeVar, Any
import numpy as np
from rj_gameplay.calculations import wall_calculations


class WallBall(play.IPlay):
    """
    Test play for the wall tactic. Directs robots to form a wall between the ball and goal.
    """
    def __init__(self, action_client_dict: Dict[Type[Any], List[Any]]):
        # defaults to walling between ball pos and goal pos
        self.wall_tactic_1 = wall_tactic.WallTactic()
        self.wall_tactic_2 = wall_tactic.WallTactic()
        self.wall_tactic_3 = wall_tactic.WallTactic()
        self.role_assigner = NaiveRoleAssignment()

        self.num_wallers = 3

    def compute_props(self, prev_props):
        pass

    def tick(
        self,
        world_state: rc.WorldState,
        prev_results: role.assignment.FlatRoleResults,
        props,
    ) -> Tuple[Dict[Type[tactic.SkillEntry], List[role.RoleRequest]], List[tactic.SkillEntry]]:

        # pre-calculate wall points and store in numpy array
        wall_pts = wall_calculations.find_wall_pts(self.num_wallers,
                                                   world_state)

        # Get role requests from all tactics and put them into a dictionary
        role_requests: play.RoleRequests = {}
        role_requests[self.wall_tactic_1] = self.wall_tactic_1.get_requests(
            world_state, wall_pts[0], None)
        role_requests[self.wall_tactic_2] = self.wall_tactic_2.get_requests(
            world_state, wall_pts[1], None)
        role_requests[self.wall_tactic_3] = self.wall_tactic_3.get_requests(
            world_state, wall_pts[2], None)

        # Flatten requests and use role assigner on them
        flat_requests = play.flatten_requests(role_requests)
        flat_results = self.role_assigner.assign_roles(flat_requests, world_state, prev_results)
        role_results = play.unflatten_results(flat_results)

        # Get list of all skills with assigned roles from tactics
        skill_dict = {}
        skills = []
        skills += self.wall_tactic_1.tick(world_state,
                                          role_results[self.wall_tactic_1])
        skill_dict.update(role_results[self.wall_tactic_1])

        skills += self.wall_tactic_2.tick(world_state,
                                          role_results[self.wall_tactic_2])
        skill_dict.update(role_results[self.wall_tactic_2])

        skills += self.wall_tactic_3.tick(world_state,
                                          role_results[self.wall_tactic_3])
        skill_dict.update(role_results[self.wall_tactic_3])

        return (skill_dict, skills)

    def is_done(self, world_state):
        # need to check this
        return self.wall_tactic_1.is_done(
            world_state) and self.wall_tactic_2.is_done(
                world_state) and self.wall_tactic_3.is_done(world_state)
