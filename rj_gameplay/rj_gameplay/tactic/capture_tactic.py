from dataclasses import dataclass
from typing import List, Optional
from typing import Dict, Generic, List, Optional, Tuple, Type, TypeVar

import stp.action as action
import stp.rc as rc
import stp.tactic as tactic
import stp.role as role

import rj_gameplay.eval
import rj_gameplay.skill as skills
from rj_gameplay.skill import shoot, capture, move
import stp.skill as skill
import numpy as np


class CaptureCost(role.CostFn):
    """
    A cost function for how to choose a striker
    TODO: Implement a better cost function
    """

    def __call__(
        self,
        robot: rc.Robot,
        prev_result: Optional["RoleResult"],
        world_state: rc.WorldState,
    ) -> float:
    
        return np.linalg.norm(world_state.ball.pos - robot.pose[0:2])

class Capture(tactic.ITactic):
    """
    A striker tactic which captures then shoots the ball
    """


    def __init__(self):
        self.capture = tactic.SkillEntry(capture.Capture())
        self.cost = CaptureCost()
        
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
        """ Checks if we have the ball and returns the proper request
        :return: A list of size 1 of role requests
        """

        role_requests: tactic.RoleRequests = {}

        capture_request = role.RoleRequest(role.Priority.HIGH, True, self.cost)
        role_requests[self.capture] = [capture_request]

        return role_requests

    def tick(self, role_results: tactic.RoleResults) -> List[tactic.SkillEntry]:
        """
        :return: A list of size 1 skill depending on which role is filled
        """
        # capture_result: tactic.RoleResults
        # capture_result = role_results[self.capture]
        # shoot_result = role_results[self.shoot]
        capture_result = role_results[self.capture]

        if capture_result and capture_result[0].is_filled():
            return [self.capture]
        return []

    def is_done(self, world_state):
        return self.capture.skill.is_done(world_state)