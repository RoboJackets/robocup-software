"""Contains the stub for the striker tactic. """

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

        return robot.pose[1] - world_state.ball.pos[1]

def marker_heuristic(point: np.array):
    return 1

class NMark(tactic.ITactic):
    """
    A tactic which creates n robots with some marking heuristic
    """
    def __init__(self, n: int):
        self.num_markers = n
        self.markers_dict = {}
        for i in range(self.num_markers):
            self.markers_dict[i] = tactic.SkillEntry(mark.Mark(marker_heuristic))
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
            role_requests[self.markers_dict[i]] = [role.RoleRequest(role.Priority.LOW, False, self.cost)]

        return role_requests

    def tick(self, role_results: tactic.RoleResults) -> List[tactic.SkillEntry]:
        """
        :return: skills for the number of markers assigned from the n markers
        """
        skills = []


        for i in range(self.num_markers):
            if role_results[self.markers_dict[i]][0]:
                # print(role_results[self.markers_dict[i]][0])
                skills.append(self.markers_dict[i])

        return skills
