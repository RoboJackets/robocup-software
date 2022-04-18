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

class PickShortestPositiveReceiver(stp.role.CostFn):
    """
    
    """

    def __init__(self):
        pass

    def __call__(self, robot: stp.rc.Robot, world_state: stp.rc.WorldState) -> float:
        dist_from_goal = lambda pos: np.linalg.norm(pos - world_state.field.their_goal_loc)
        dist_from_ball = lambda pos: np.linalg.norm(pos - world_state.ball.pos)

        robot_to_goal_dist = dist_from_goal(robot.pose[0:2])
        robot_to_ball_dist = dist_from_ball(robot.pose[0:2])
        ball_to_goal_dist = dist_from_goal(world_state.ball.pos)

        return_val = 1e9

        if world_state.ball.pos[1] > 8:
            if robot_to_goal_dist < 3.5:
                return_val = np.abs(robot.pose[0])
        elif robot_to_goal_dist < ball_to_goal_dist:
            return_val = robot_to_ball_dist
        
        print("Receiver Cost - ", robot.id, " : ", return_val)
        return return_val
