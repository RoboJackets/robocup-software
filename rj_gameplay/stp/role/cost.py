"""This module contains a variety of functions that return cost functions for
convenience."""

from typing import Optional

import numpy as np
import stp.rc
import stp.role


class PickRobotById(stp.role.CostFn):
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

    # TODO: rm this from stp/role/__init__.py
    def unassigned_cost_fn(
        self, prev_results: Optional["RoleResult"], world_state: stp.rc.WorldState
    ) -> float:
        pass

    # TODO: enforce __repr__ for all STP classes?
    #       using common fmt "clsname(init fields=...)"
    def __repr__(self):
        return f"PickRobotById(robot={self._robot_id})"


class PickClosestRobot(stp.role.CostFn):
    def __init__(self, target_point):
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
        return f"PickClosestRobot(target_point={self._target_point})"
