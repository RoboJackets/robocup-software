from typing import Optional

import numpy as np

import stp.role as role
import stp.game_state as game_state


def flat_switch_cost(
    robot: game_state.Robot, prev_robot: Optional[game_state.Robot], switch_cost: float
) -> float:
    if prev_robot:
        return switch_cost * (robot.id != prev_robot.id)
    else:
        return 0


def distance_to_pt(
    pt: np.ndarray, saturate_dist: float, switch_cost: float
) -> role.CostFn:
    """Creates a cost function that returns the distance from the robot's position to a
    point, saturating at the passed in distance.
    :param pt: Point to evaluate robot's position to.
    :param saturate_dist: Distance at which the cost function saturates.
    :param switch_cost: Flat cost added for switching to a different robot.
    :return: Cost function
    """

    def cost_fn(robot: game_state.Robot, prev_robot: Optional[game_state.Robot]):
        dist: float = np.linalg.norm(robot.pose[:2] - pt)
        cost: float = dist / saturate_dist

        # Add the cost for switching robots.
        cost += flat_switch_cost(robot, prev_robot, switch_cost)

        return max(min(cost, 1.0), 0.0)

    return cost_fn


def constant(value: float, switch_cost: float) -> role.CostFn:
    """Cost function that returns a constant.
    :param value: Constant value of the cost function.
    :param switch_cost: Flat cost added for switching to a different robot.
    :return: Cost function.
    """
    assert 0.0 <= value <= 1.0

    def cost_fn(robot: game_state.Robot, prev_robot: Optional[game_state.Robot]):
        cost = value

        # Add the cost for switching robots.
        cost += flat_switch_cost(robot, prev_robot, switch_cost)

        return max(min(cost, 1.0), 0.0)

    return cost_fn


def zero(switch_cost: float) -> role.CostFn:
    """Cost function that returns 0.
    :param switch_cost: Flat cost added for switching to a different robot.
    :return: Cost function that returns 0.
    """
    return constant(0.0, switch_cost)


def one() -> role.CostFn:
    """Cost function that returns 1.
    :return: Cost function that returns 1.
    """
    return constant(1.0, 0.0)
