"""Contains the stub for the striker tactic. """

from dataclasses import dataclass
from typing import List, Optional
from typing import Dict, Generic, List, Optional, Tuple, Type, TypeVar

import stp.action as action
import stp.rc as rc
import stp.tactic as tactic
import stp.role as role

import rj_gameplay.eval
import rj_gameplay.skill as skills
from rj_gameplay.skill import shoot, capture
import stp.skill as skill


class striker_cost(role.CostFn):
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

        return robot.pose[0] - world_state.ball.pos[0]

class Striker(tactic.ITactic):
    """
    A striker tactic which captures then shoots the ball
    """

    def __init__(self):
        self.capture = tactic.SkillEntry(skills.capture.Capture())
        self.shoot = tactic.SkillEntry(skills.shoot.Shoot())
        self.cost = striker_cost()
        
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

        striker_request = role.RoleRequest(role.Priority.HIGH, True, self.cost)
        has_ball = True
        for robot in world_state.our_robots:
            if robot.has_ball_sense:
                has_ball = True
        if has_ball:
            role_requests[self.shoot] = [striker_request]
            role_requests[self.capture] = []
        else:
            role_requests[self.capture] = [striker_request]
            role_requests[self.shoot] = []
        role_requests

        return role_requests

    def tick(self, role_results: tactic.RoleResults) -> List[tactic.SkillEntry]:
        """
        :return: A list of size 1 skill depending on which role is filled
        """
        capture_result: tactic.RoleResults
        capture_result = role_results[self.capture]
        shoot_result = role_results[self.shoot]

        if capture_result and capture_result[0].is_filled():
            return [self.capture]
        if shoot_result and shoot_result[0].is_filled():
            return [self.shoot]
        return []