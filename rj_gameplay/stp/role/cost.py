"""This module contains a variety of cost functions for convenience."""

from typing import Any

import numpy as np

import stp.rc
import stp.role


class PickRobotById(stp.role.CostFn):
    """Always select robot of robot_id (passed on init)."""

    def __init__(self, robot_id: int):
        self._robot_id = robot_id

    def __call__(
        self,
        robot: stp.rc.Robot,
        world_state: stp.rc.WorldState,
    ) -> float:
        if robot.id == self._robot_id:
            return 0.0

        # TODO: use max int or float('inf')
        return 1e9

    # TODO: rm this as it is not needed in new architecture
    #       (costFns are linked to Roles, implying that each costFn is the cost
    #        of a specific robot filling a specific Role -> "unassigned cost" = ??)
    def unassigned_cost_fn(
        self, prev_results: Any, world_state: stp.rc.WorldState
    ) -> float:
        pass

    # TODO: enforce __repr__ for all STP classes?
    #       using common fmt "clsname(init fields=...)"
    def __repr__(self):
        return f"PickRobotById(robot={self._robot_id})"


class PickClosestToPoint(stp.role.CostFn):
    """Always select closest robot to some target_point (passed on init).
    Can get closest to ball by passing in `world_state.ball.pos`.
    """

    def __init__(self, target_point: np.ndarray):
        self._target_point = target_point

    def __call__(
        self,
        robot: stp.rc.Robot,
        world_state: stp.rc.WorldState,
    ) -> float:
        if world_state is not None and robot in world_state.our_robots:
            return np.linalg.norm(robot.pose[:2] - self._target_point)

        # TODO: use max int or float('inf')
        return 1e9

    def __repr__(self):
        return f"PickClosestToPoint(target_point={self._target_point})"


class PickFarthestFromPoint(stp.role.CostFn):
    """Always select farthest robot to some target_point (passed on init).
    Can get farthest from ball by passing in `world_state.ball.pos`.
    """

    def __init__(self, target_point):
        self.closest_picker = PickClosestToPoint(target_point)

    def __call__(
        self,
        robot: stp.rc.Robot,
        world_state: stp.rc.WorldState,
    ) -> float:
        closest_cost = self.closest_picker(robot, world_state)
        if closest_cost >= 1e9:
            return 1e9
        return -closest_cost

    def __repr__(self):
        return f"PickFarthestFromPoint(target_point={self._target_point})"


class PickClosestInFront(stp.role.CostFn):
    """Select closest robot "in front" of the current one. (Assuming opp goal is front.)"""

    def __init__(self, target_point):
        self._target_point = target_point
        self.closest_picker = PickClosestToPoint(target_point)

    def __call__(
        self,
        robot: stp.rc.Robot,
        world_state: stp.rc.WorldState,
    ) -> float:
        closest_cost = self.closest_picker(robot, world_state)
        bot_to_goal = np.linalg.norm(robot.pose[0:2] - world_state.field.their_goal_loc)
        ball_to_goal = np.linalg.norm(
            world_state.ball.pos - world_state.field.their_goal_loc
        )

        if closest_cost >= 1e9:
            return 1e9
        if bot_to_goal > ball_to_goal:
            return closest_cost + 100
        return closest_cost

    def __repr__(self):
        return f"PickClosestInFront(target_point={self._target_point})"
