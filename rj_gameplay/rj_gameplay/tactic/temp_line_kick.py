"""Temp tactic to test LineKick skill."""

from dataclasses import dataclass
from typing import List, Optional
from typing import Dict, Generic, List, Optional, Tuple, Type, TypeVar

import stp.action as action
import stp.rc as rc
import stp.tactic as tactic
import stp.role as role

import rj_gameplay.eval
import rj_gameplay.skill as skills
# from rj_gameplay.skill import shoot, capture, move
from rj_gameplay.skill import line_kick 
import stp.skill as skill
import numpy as np


class move_cost(role.CostFn):
    """
    A cost function for how to choose a striker
    TODO: Implement a better cost function
    """
    def __init__(self, target_point : np.ndarray):
        self.target_point = target_point

    def __call__(
        self,
        robot: rc.Robot,
        prev_result: Optional["RoleResult"],
        world_state: rc.WorldState,
    ) -> float:
    
        # raw dist
        return (robot.pose[0] - self.target_point[0])**2 + (robot.pose[1] - self.target_point[1])**2

class LineKick(tactic.ITactic):
    """LineKick skill wrapper"""

    def __init__(self, world_state: rc.WorldState):
        self.line_kick = tactic.SkillEntry(line_kick.LineKick(None, world_state = world_state))
        # TODO: base on the target of the tactic?
        self.cost = move_cost(world_state.ball.pos)
        # self.cost = move_cost(self.line_kick.move.target_point)
        
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
        :return: A list of size 1 of role requests
        """

        role_requests: tactic.RoleRequests = {}

        line_kick_request = role.RoleRequest(role.Priority.HIGH, True, self.cost)
        role_requests[self.line_kick] = [line_kick_request]

        return role_requests

    def tick(self, role_results: tactic.RoleResults) -> List[tactic.SkillEntry]:
        """
        :return: A list of size 1 skill depending on which role is filled
        """
        line_kick_result = role_results[self.line_kick]

        if line_kick_result and line_kick_result[0].is_filled():
            return [self.line_kick]
        return []

    def is_done(self, world_state):
        return self.line_kick.skill.is_done(world_state)
