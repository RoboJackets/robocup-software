"""Tactic to test the mark skill. """

from dataclasses import dataclass
from typing import List, Optional

import stp.action as action
import stp.rc as rc
import stp.tactic as tactic
import stp.role as role

import rj_gameplay.eval
import rj_gameplay.skill as skills
from rj_gameplay.skill import mark
import stp.skill as skill

import numpy as np

class marker_cost(role.CostFn):
    """
    A cost function for how to choose a marker
    TODO: Implement a better cost function
    """

    def __call__(
        self,
        robot: rc.Robot,
        prev_result: Optional["RoleResult"],
        world_state: rc.WorldState,
    ) -> float:

        # TODO: make it not just robot 7 that marks 
        if robot.id == 7:
            return 0.0
        return 1.0

    def unassigned_cost_fn(
        self,
        prev_result: Optional["RoleResult"],
        world_state: rc.WorldState,
    ) -> float:

        #TODO: Implement real unassigned cost function
        return 9999

def marker_heuristic(point: np.array):
    # TODO: use with CostBehavior
    return -1

class TestMarkTactic(tactic.ITactic):
    """
    A tactic which creates n robots with some marking heuristic
    """
    def __init__(self, n: int):
        self.num_markers = n
        self.markers_list = []
        for i in range(self.num_markers):
            self.markers_list.append(tactic.SkillEntry(mark.Mark(None, None)))
        self.cost = marker_cost()
        
    def compute_props(self):
        pass


    def create_request(self, **kwargs) -> role.RoleRequest:
        """Creates a sane default RoleRequest.
        :return: A list of size 1 of a sane default RoleRequest.
        """
        pass

    def get_requests(
        self, world_state: rc.WorldState, props
    ) -> List[tactic.RoleRequests]:
        """
        :return: role request for n markers
        """

        role_requests = {}

        for i in range(self.num_markers):
            role_requests[self.markers_list[i]] = [role.RoleRequest(role.Priority.LOW, False, self.cost)]

        return role_requests

    def tick(self, role_results: tactic.RoleResults) -> List[tactic.SkillEntry]:
        """
        :return: skills for the number of markers assigned from the n markers
        """
        skills = []

        for i in range(self.num_markers):
            if role_results[self.markers_list[i]][0]:
                skills.append(self.markers_list[i])

        return skills

    def is_done(self, world_state):
        for mark_skill in self.markers_list:
            if not mark_skill.skill.is_done(world_state):
                return False
        return True
