import numpy as np
import stp.rc as rc
import stp.role as role
import stp.role.cost as cost


def create_ball() -> rc.Ball:
    """Convenience function for creating a rc.Ball at (0, 0) with (0,0) velocity."""
    return rc.Ball(np.zeros(2), np.zeros(2))


def test_constant() -> None:
    """Tests cost.constant for both the same robot and different robot cases."""
    switch_cost = 0.5
    cost_fn = cost.constant(0.5, switch_cost)

    robot1 = rc.Robot(1, np.array([0, 0, 0]), np.array([0, 0, 0]), False)
    robot2 = rc.Robot(2, np.array([0, 0, 0]), np.array([0, 0, 0]), False)
    stub_request = role.RoleRequest(role.Priority.HIGH, True, None)
    robot1_role_result = role.RoleResult(stub_request, 0.0, role.Role(robot1))
    world_state = rc.WorldState([robot1, robot2], [], create_ball())

    cost_none: float = cost_fn(robot1, None, world_state)
    assert cost_none == 0.5

    cost_same: float = cost_fn(robot1, robot1_role_result, world_state)
    assert cost_same == 0.5

    cost_switch: float = cost_fn(robot2, robot1_role_result, world_state)
    assert cost_switch == 1.0


def test_distance_to_pt() -> None:
    """Tests cost.distance_to_pt for both the same robot and different robot cases."""
    switch_cost = 0.5
    saturate_dist = 1.0
    target_pt = np.array([1.0, 2.0])
    cost_fn = cost.distance_to_pt(target_pt, saturate_dist, switch_cost)

    robot1 = rc.Robot(1, np.array([1.0, 2.0, 0]), np.array([0, 0, 0]), False)
    robot2 = rc.Robot(2, np.array([2.0, 3.0, 0]), np.array([0, 0, 0]), False)
    stub_request = role.RoleRequest(role.Priority.HIGH, True, None)
    robot1_role_result = role.RoleResult(stub_request, 0.0, role.Role(robot1))
    world_state = rc.WorldState([robot1, robot2], [], create_ball())

    cost_none: float = cost_fn(robot1, None, world_state)
    assert cost_none == 0.0

    cost_same: float = cost_fn(robot1, robot1_role_result, world_state)
    assert cost_same == 0.0

    cost_robot2: float = cost_fn(robot2, None, world_state)
    assert cost_robot2 == 1.0

    cost_robot2_switch: float = cost_fn(robot2, robot1_role_result, world_state)
    assert cost_robot2_switch == 1.0
