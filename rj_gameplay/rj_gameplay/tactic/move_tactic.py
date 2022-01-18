"""Contains the stub for the move tactic. """

from dataclasses import dataclass
from typing import List, Optional
from typing import Dict, Generic, List, Optional, Tuple, Type, TypeVar

import stp.action as action
import stp.rc as rc
import stp.tactic as tactic
import stp.role as role

import rj_gameplay.eval
import rj_gameplay.skill as skills
from rj_gameplay.skill import move
import stp.skill as skill
import numpy as np

MAX_ROBOT_VELOCITY = 3.0


class move_cost(role.CostFn):
    """
    A cost function for how to choose a striker
    TODO: Implement a better cost function
    """

    def __init__(self, target_point: np.ndarray, cost_scale: float = 1.0):
        self.target_point = target_point
        self.cost_scale = cost_scale

    def __call__(
        self,
        robot: rc.Robot,
        prev_result: Optional["RoleResult"],
        world_state: rc.WorldState,
    ) -> float:
        return (
            (
                (robot.pose[0] - self.target_point[0]) ** 2
                + (robot.pose[1] - self.target_point[1]) ** 2
            )
            / MAX_ROBOT_VELOCITY
            * self.cost_scale
        )

    def unassigned_cost_fn(
        self,
        prev_result: Optional["RoleResult"],
        world_state: rc.WorldState,
    ) -> float:

        return role.BIG_STUPID_NUMBER_CONST_FOR_UNASSIGNED_COST_PLS_CHANGE


class Move(tactic.ITactic):

    def __init__(
        self,
        target_point: np.ndarray,
        face_point: np.ndarray = None,
        cost_scale: float = 1.0,
        priority: role.Priority = role.Priority.MEDIUM,
    ):
        self.move = tactic.SkillEntry(
            move.Move(target_point=target_point, face_point=face_point)
        )
        self.cost = move_cost(target_point, cost_scale=cost_scale)
        self.priority = priority

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
        """Checks if we have the ball and returns the proper request
        :return: A list of size 1 of role requests
        """

        role_requests: tactic.RoleRequests = {}

        move_request = role.RoleRequest(self.priority, True, self.cost)
        role_requests[self.move] = [move_request]

        return role_requests

    def tick(
        self, world_state: rc.WorldState, role_results: tactic.RoleResults
    ) -> List[tactic.SkillEntry]:
        """
        :return: A list of size 1 skill depending on which role is filled
        """
        move_result = role_results[self.move]

        if move_result and move_result[0].is_filled():
            return [self.move]
        return []

    def is_done(self, world_state):
        return self.move.skill.is_done(world_state)
