"""This module contains a variety of functions that return cost functions for
convenience."""

from typing import Optional

import numpy as np
import stp.rc as rc
import stp.role as role


def flat_switch_cost(
    robot: rc.Robot, prev_robot: Optional[rc.Robot], switch_cost: float
) -> float:
    """Returns the cost of switching from prev_robot to robot, ie. switch_cost if
    robot.id != prev_robot.id, otherwise 0.
    :param robot: The currently selected robot.
    :param prev_robot: The previously selected robot.
    :param switch_cost: The cost of switching to a different robot.
    :return: switch_cost if robot.id != prev_robot.id, otherwise returns 0.
    """
    if prev_robot:
        return switch_cost * (robot.id != prev_robot.id)

    return 0


def distance_to_pt(
    eval_pt: np.ndarray, saturate_dist: float, switch_cost: float
) -> role.CostFn:
    """Creates a cost function that returns the distance from the robot's position to a
    point, saturating at the passed in distance.
    :param eval_pt: Point to evaluate robot's position to.
    :param saturate_dist: Distance at which the cost function saturates.
    :param switch_cost: Flat cost added for switching to a different robot.
    :return: Cost function
    """

    def cost_fn(
        robot: rc.Robot,
        prev_result: Optional[role.RoleResult],
        world_state: rc.WorldState,
    ):
        dist: float = np.linalg.norm(robot.pose[:2] - eval_pt)
        cost: float = min(dist, saturate_dist)

        # Add the cost for switching robots.
        if prev_result and prev_result.role.is_filled():
            cost += flat_switch_cost(
                robot, prev_result.role.robot, switch_cost
            )

        return cost

    return cost_fn


def constant(value: float, switch_cost: float) -> role.CostFn:
    """Cost function that returns a constant.
    :param value: Constant value of the cost function.
    :param switch_cost: Flat cost added for switching to a different robot.
    :return: Cost function.
    """

    def cost_fn(
        robot: rc.Robot,
        prev_result: Optional[role.RoleResult],
        world_state: rc.WorldState,
    ):
        cost = value

        # Add the cost for switching robots.
        if prev_result and prev_result.role.is_filled():
            cost += flat_switch_cost(
                robot, prev_result.role.robot, switch_cost
            )

        return cost

    return cost_fn


def zero(switch_cost: float) -> role.CostFn:
    """Cost function that returns 0.
    :param switch_cost: Flat cost added for switching to a different robot.
    :return: Cost function that returns 0.
    """
    return constant(0.0, switch_cost)
