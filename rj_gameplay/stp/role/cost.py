"""This module contains a variety of cost functions for convenience."""

from typing import Optional

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


class PickClosestRobot(stp.role.CostFn):
    """Always select closest robot to some target_point (passed on init)."""

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
