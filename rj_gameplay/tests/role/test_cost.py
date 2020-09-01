import numpy as np

import sheen.role.cost as cost
from sheen.game_state import Robot


def test_constant() -> None:
    """Tests cost.constant for both the same robot and different robot cases."""
    switch_cost = 0.5
    cost_fn = cost.constant(0.5, switch_cost)

    robot1 = Robot(1, np.array([0, 0, 0]), np.array([0, 0, 0]), False)
    robot2 = Robot(2, np.array([0, 0, 0]), np.array([0, 0, 0]), False)

    cost_none: float = cost_fn(robot1, None)
    assert cost_none == 0.5

    cost_same: float = cost_fn(robot1, robot1)
    assert cost_same == 0.5

    cost_switch: float = cost_fn(robot2, robot1)
    assert cost_switch == 1.0


def test_distance_to_pt() -> None:
    """Tests cost.distance_to_pt for both the same robot and different robot cases."""
    switch_cost = 0.5
    target_pt = np.ndarray([1.0, 2.0])
    cost_fn = cost.distance_to_pt(target_pt, switch_cost)

    robot1 = Robot(1, np.array([1.0, 2.0, 0]), np.array([0, 0, 0]), False)
    robot2 = Robot(2, np.array([2.0, 3.0, 0]), np.array([0, 0, 0]), False)

    cost_none: float = cost_fn(robot1, None)
    assert cost_none == 0.0

    cost_same: float = cost_fn(robot1, robot1)
    assert cost_same == 0.0

    cost_robot2: float = cost_fn(robot2, None)
    assert cost_robot2 == np.sqrt(2)

    cost_robot2_switch: float = cost_fn(robot2, robot1)
    assert cost_robot2_switch == 1.0
