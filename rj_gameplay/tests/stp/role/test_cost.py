import numpy as np
import stp.rc as rc
import stp.role as role
import stp.role.cost as cost
import stp.testing as testing


def test_constant() -> None:
    """Tests cost.constant for both the same robot and different robot cases."""
    switch_cost = 0.5
    cost_fn = cost.constant(0.5, switch_cost)

    robot1 = testing.generate_test_robot(robot_id=1)
    robot2 = testing.generate_test_robot(robot_id=2)
    # TODO change priority float to something useful
    stub_request = role.RoleRequest(1.0, True, None)
    robot1_role_result = role.RoleResult(stub_request, 0.0, role.Role(robot1))
    world_state = testing.generate_test_worldstate(our_robots=[robot1, robot2])

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

    robot1 = testing.generate_test_robot(robot_id=1,
                                         is_ours=True,
                                         pose=np.array([1.0, 2.0, 0]),
                                         twist=np.array([0, 0, 0]))
    robot2 = testing.generate_test_robot(robot_id=2,
                                         is_ours=True,
                                         pose=np.array([2.0, 3.0, 0]),
                                         twist=np.array([0, 0, 0]))

    # TODO change priority float to something useful
    stub_request = role.RoleRequest(1.0, True, None)
    robot1_role_result = role.RoleResult(stub_request, 0.0, role.Role(robot1))
    world_state = testing.generate_test_worldstate(our_robots=[robot1, robot2])

    cost_none: float = cost_fn(robot1, None, world_state)
    assert cost_none == 0.0

    cost_same: float = cost_fn(robot1, robot1_role_result, world_state)
    assert cost_same == 0.0

    cost_robot2: float = cost_fn(robot2, None, world_state)
    assert cost_robot2 == 1.0

    cost_robot2_switch: float = cost_fn(robot2, robot1_role_result, world_state)
    assert cost_robot2_switch == 1.5
