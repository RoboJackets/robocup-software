import math
from typing import List

import numpy as np
import stp.play as play
import stp.role as role
import stp.role.constraint as constraint
import stp.role.cost as cost
import stp.skill as skill
import stp.tactic as tactic
import stp.testing as testing
from stp import action as action
from stp.rc import Ball, Robot, WorldState
from stp.role import Priority
from stp.role.assignment import FlatRoleRequests, RoleId
from stp.role.assignment.naive import NaiveRoleAssignment, SortedRequests


class SkillBase(skill.ISkill):
    def define(self):
        pass

    def create_request(self) -> role.RoleRequest:
        switch_cost = 0.0
        return role.RoleRequest(
            Priority.LOW, required=True, cost_fn=cost.constant(0.5, switch_cost)
        )

    def __repr__(self) -> str:
        return "<{} object>".format(self.__class__.__name__)


class SkillA(SkillBase):
    ...


class SkillB(SkillBase):
    ...


class SkillC(SkillBase):
    ...


class BallSkill(SkillBase):
    ...


class Skills(tactic.SkillsEnum):
    A1 = tactic.SkillEntry(SkillA)
    A2 = tactic.SkillEntry(SkillA)
    B1 = tactic.SkillEntry(SkillB)
    B2 = tactic.SkillEntry(SkillB)
    C1 = tactic.SkillEntry(SkillC)
    C2 = tactic.SkillEntry(SkillC)
    BALL_SKILL = tactic.SkillEntry(BallSkill)


class TacticBase(tactic.ITactic[None]):
    def __init__(self, ctx: tactic.Ctx):
        self.skills = Skills(ctx.skill_factory)

        self.A1 = self.skills.A1
        self.A2 = self.skills.A2
        self.B1 = self.skills.B1
        self.B2 = self.skills.B2
        self.C1 = self.skills.C1
        self.C2 = self.skills.C2
        self.BALL_SKILL = self.skills.BALL_SKILL

    def compute_props(self, prev_props: None) -> None:
        return None

    def tick(
        self, role_results: tactic.RoleResults, props: None
    ) -> List[action.IAction]:
        # Dummy tick function doesn't return any actions.
        return []

    def get_requests(self, world_state: WorldState, props: None) -> tactic.RoleRequests:
        role_requests: tactic.RoleRequests = {
            self.A1: [self.A1.skill.create_request().with_priority(Priority.LOW)],
            self.A2: [self.A2.skill.create_request().with_priority(Priority.MEDIUM)],
            self.B1: [self.B1.skill.create_request().with_priority(Priority.MEDIUM)],
            self.B2: [self.B2.skill.create_request().with_priority(Priority.HIGH)],
            self.C1: [self.C1.skill.create_request().with_priority(Priority.LOW)],
            self.C2: [self.C2.skill.create_request().with_priority(Priority.MEDIUM)],
            self.BALL_SKILL: [
                self.BALL_SKILL.skill.create_request()
                .with_priority(Priority.HIGH)
                .with_constraint_fn(constraint.has_ball())
            ],
        }

        return role_requests


def get_simple_role_ids() -> List[RoleId]:
    """Creates and returns a list of role ids with skills SkillA, SkillB and SkillC for
    TacticBase.
    :return: List of role ids with skills SkillA, SkillB and SkillC for TacticBase.
    """

    skill_entries = [
        tactic.SkillEntry(SkillA),
        tactic.SkillEntry(SkillB),
        tactic.SkillEntry(SkillC),
        tactic.SkillEntry(BallSkill),
    ]
    skill_instances = [SkillA(), SkillB(), SkillC(), BallSkill()]

    for idx, (skill_entry, skill_instance) in enumerate(
        zip(skill_entries, skill_instances)
    ):
        skill_entry.set_idx(idx)
        skill_entry.skill = skill_instance

    return [(TacticBase, skill_entry, 0) for skill_entry in skill_entries]


def test_get_sorted_requests_simple():
    """Manually create a Requests and check that get_sorted_requests returns a list of
    three dictionaries, one for each priority level.
    """
    role_id_a, role_id_b, role_id_c, role_id_ball = get_simple_role_ids()

    switch_cost = 0.0
    constant_cost = cost.constant(0.5, switch_cost)

    requests: FlatRoleRequests = {
        role_id_a: role.RoleRequest(
            Priority.HIGH, required=True, cost_fn=constant_cost
        ),
        role_id_b: role.RoleRequest(Priority.LOW, required=True, cost_fn=constant_cost),
        role_id_c: role.RoleRequest(
            Priority.MEDIUM, required=True, cost_fn=constant_cost
        ),
    }

    # Get the sorted requests.
    sorted_requests = NaiveRoleAssignment.get_sorted_requests(requests)

    # Check that the lengths of the sorted_requests is correct.
    assert len(sorted_requests) == 3
    assert len(sorted_requests[0]) == 1
    assert len(sorted_requests[1]) == 1
    assert len(sorted_requests[2]) == 1

    # Check that A is in high priority, B is in low priority, C is in medium priority.
    assert role_id_a in sorted_requests[Priority.HIGH]
    assert role_id_b in sorted_requests[Priority.LOW]
    assert role_id_c in sorted_requests[Priority.MEDIUM]

    # Check that each of the role requests are equal.
    assert sorted_requests[Priority.LOW][role_id_b] == requests[role_id_b]
    assert sorted_requests[Priority.MEDIUM][role_id_c] == requests[role_id_c]
    assert sorted_requests[Priority.HIGH][role_id_a] == requests[role_id_a]


def get_tactic_ctx() -> tactic.Ctx:
    """Creates a simple tactic context for convenience.
    :return: Tactic context containing SkillA, SkillB and SkillC.
    """
    skill_registry = skill.Registry()

    skill_registry[SkillA] = SkillA()
    skill_registry[SkillB] = SkillB()
    skill_registry[SkillC] = SkillC()
    skill_registry[BallSkill] = BallSkill()

    skill_factory = skill.Factory(skill_registry)
    return tactic.Ctx(skill_factory)


def test_get_sorted_requests_multiple() -> None:
    """Tests get_sorted_requests with a more complicated example."""
    tactic_ctx = get_tactic_ctx()
    tactic_instance = TacticBase(tactic_ctx)

    world_state: WorldState = testing.generate_test_worldstate()

    requests: play.RoleRequests = {
        TacticBase: tactic_instance.get_requests(world_state, None)
    }

    flat_requests: FlatRoleRequests = play.flatten_requests(requests)

    # Flat requests contains:
    #  A1: LOW,    A2: MEDIUM
    #  B1: MEDIUM, B2: HIGH,
    #  C1: LOW,    C2: MEDIUM

    sorted_requests: SortedRequests = NaiveRoleAssignment.get_sorted_requests(
        flat_requests
    )

    # Check the lengths of each dictionary.
    low_tactics = [tactic_instance.A1, tactic_instance.C1]
    med_tactics = [tactic_instance.A2, tactic_instance.B1, tactic_instance.C2]
    hi_tactics = [tactic_instance.B2, tactic_instance.BALL_SKILL]

    assert len(sorted_requests) == 3
    assert len(sorted_requests[Priority.LOW]) == len(low_tactics)
    assert len(sorted_requests[Priority.MEDIUM]) == len(med_tactics)
    assert len(sorted_requests[Priority.HIGH]) == len(hi_tactics)

    for low_tactic in low_tactics:
        assert (TacticBase, low_tactic, 0) in sorted_requests[Priority.LOW]
        assert (
            sorted_requests[Priority.LOW][TacticBase, low_tactic, 0]
            == requests[TacticBase][low_tactic][0]
        )

    for med_tactic in med_tactics:
        assert (TacticBase, med_tactic, 0) in sorted_requests[Priority.MEDIUM]
        assert (
            sorted_requests[Priority.MEDIUM][TacticBase, med_tactic, 0]
            == requests[TacticBase][med_tactic][0]
        )

    for hi_tactic in hi_tactics:
        assert (TacticBase, hi_tactic, 0) in sorted_requests[Priority.HIGH]
        assert (
            sorted_requests[Priority.HIGH][TacticBase, hi_tactic, 0]
            == requests[TacticBase][hi_tactic][0]
        )


def test_compute_costs_matrix() -> None:
    """Tests the compute_costs_matrix function.

    Costs:
        A: Dist to (0, 0)
        B: Dist to (1, 1)
        C: Dist to (2, 2)
    """

    # Get the three roles.
    role_id_a, role_id_b, role_id_c, role_id_ball = get_simple_role_ids()

    # Create the cost functions.
    switch_cost = 0.0
    cost_a = cost.distance_to_pt(np.array([0, 0]), math.sqrt(8), switch_cost)
    cost_b = cost.distance_to_pt(np.array([1, 1]), math.sqrt(8), switch_cost)
    cost_c = cost.distance_to_pt(np.array([2, 2]), math.sqrt(8), switch_cost)

    # Create the requests of same priority.
    requests: FlatRoleRequests = {
        role_id_a: role.RoleRequest(Priority.LOW, required=True, cost_fn=cost_a),
        role_id_b: role.RoleRequest(Priority.LOW, required=True, cost_fn=cost_b),
        role_id_c: role.RoleRequest(Priority.LOW, required=True, cost_fn=cost_c),
    }

    # Create the robots at (0, 0), (1, 1), (2, 2), (3, 3)
    free_robots = np.array([
        testing.generate_test_robot(robot_id=1, pose=np.array([0, 0, 0])),
        testing.generate_test_robot(robot_id=2, pose=np.array([1, 1, 0])),
        testing.generate_test_robot(robot_id=3, pose=np.array([2, 2, 0])),
        testing.generate_test_robot(robot_id=4, pose=np.array([3, 3, 0])),
    ])

    # Construct the world state.
    out_robots: List[Robot] = list(free_robots)
    their_robots: List[Robot] = []
    ball: Ball = testing.generate_test_ball()

    world_state: WorldState = testing.generate_test_worldstate(
        our_robots=out_robots)
    prev_results = {}

    # Compute the cost matrix.
    costs_matrix: np.ndarray = NaiveRoleAssignment.compute_costs_matrix(
        free_robots, requests, world_state, prev_results
    )

    # Check that the cost matrix is of the right size, ie. (num_robots, num_requests).
    assert costs_matrix.shape == (4, 3)

    # fmt: off
    expected_costs_matrix = np.array(
        [[0.0,              math.sqrt(2),       math.sqrt(8)],
         [math.sqrt(2),              0.0,       math.sqrt(2)],
         [math.sqrt(8),     math.sqrt(2),                0.0],
         [math.sqrt(8),     math.sqrt(8),       math.sqrt(2)]]
    )
    # fmt: on

    # costs_matrix should be equal to expected_costs_matrix.
    assert np.allclose(costs_matrix, expected_costs_matrix)


def test_assign_prioritized_roles() -> None:
    """Tests that for the role requests and free robots above that role assignment
    returns the expected result.
    """
    # Get the three roles.
    role_id_a, role_id_b, role_id_c, role_id_ball = get_simple_role_ids()

    # Create the cost functions.
    switch_cost = 0.0
    cost_a = cost.distance_to_pt(np.array([0, 0]), math.sqrt(8), switch_cost)
    cost_b = cost.distance_to_pt(np.array([1, 1]), math.sqrt(8), switch_cost)
    cost_c = cost.distance_to_pt(np.array([2, 2]), math.sqrt(8), switch_cost)

    # Create the requests of same priority.
    requests: FlatRoleRequests = {
        role_id_a: role.RoleRequest(Priority.LOW, required=True, cost_fn=cost_a),
        role_id_b: role.RoleRequest(Priority.LOW, required=True, cost_fn=cost_b),
        role_id_c: role.RoleRequest(Priority.LOW, required=True, cost_fn=cost_c),
    }

    # Create the robots at (0, 0), (1, 1), (2, 2), (3, 3)
    free_robots = np.array([
        testing.generate_test_robot(robot_id=0, pose=np.array([0, 0, 0])),
        testing.generate_test_robot(robot_id=0, pose=np.array([1, 1, 0])),
        testing.generate_test_robot(robot_id=0, pose=np.array([2, 2, 0])),
        testing.generate_test_robot(robot_id=0, pose=np.array([3, 3, 0])),
    ])

    # Construct the world state.
    our_bots: List[Robot] = list(free_robots)
    their_bots: List[Robot] = []

    world_state: WorldState = testing.generate_test_worldstate(
        our_robots=our_bots, their_robots=their_bots)

    # Assign the roles.
    results, new_free_robots = NaiveRoleAssignment.assign_prioritized_roles(
        requests, world_state, free_robots, {}
    )

    # Check that the the three role ids are assigned.
    assert len(results) == 3
    assert role_id_a in results
    assert role_id_b in results
    assert role_id_c in results

    # Check that A->0, B->1, C->2.
    assert results[role_id_a].role.robot == free_robots[0]
    assert results[role_id_b].role.robot == free_robots[1]
    assert results[role_id_c].role.robot == free_robots[2]

    # Check that the costs for each role result are 0.
    assert results[role_id_a].cost == 0.0
    assert results[role_id_b].cost == 0.0
    assert results[role_id_c].cost == 0.0

    # Check that new_free_robots is length 1 and contains free_robots[3].
    assert len(new_free_robots) == 1
    assert new_free_robots[0] == free_robots[3]


def test_assign_roles() -> None:
    """Tests that NaiveRoleAssignment.assign_roles assigns HIGH, then MEDIUM, then LOW
    priority. This is tested by having a MEDIUM priority role request that has a lower
    cost than a HIGH priority role request, and expecting that the HIGH role request is
    fulfilled first.
    """

    # Get the three roles.
    role_id_a, role_id_b, role_id_c, role_id_ball = get_simple_role_ids()

    # Create the cost functions.
    switch_cost = 0.0
    cost_a = cost.distance_to_pt(np.array([0, 0]), math.sqrt(8), switch_cost)
    cost_b = cost.distance_to_pt(np.array([1, 1]), math.sqrt(8), switch_cost)
    cost_c = cost.distance_to_pt(np.array([2, 2]), math.sqrt(8), switch_cost)

    # Create the requests in descending priority.
    requests: FlatRoleRequests = {
        role_id_a: role.RoleRequest(Priority.HIGH, required=True, cost_fn=cost_a),
        role_id_b: role.RoleRequest(Priority.MEDIUM, required=True, cost_fn=cost_b),
        role_id_c: role.RoleRequest(Priority.LOW, required=True, cost_fn=cost_c),
    }

    # Create the robots at (1, 1), (2, 2), (3, 3), (4, 4).
    free_robots = np.array([
        testing.generate_test_robot(robot_id=0, pose=np.array([1, 1, 0])),
        testing.generate_test_robot(robot_id=1, pose=np.array([2, 2, 0])),
        testing.generate_test_robot(robot_id=2, pose=np.array([3, 3, 0])),
        testing.generate_test_robot(robot_id=3, pose=np.array([4, 4, 0])),
    ])



    # Construct the world state.
    out_robots: List[Robot] = list(free_robots)
    their_robots: List[Robot] = []

    world_state: WorldState = testing.generate_test_worldstate(
        our_robots=out_robots, their_robots=their_robots)

    # Assign the roles.
    results = NaiveRoleAssignment.assign_roles(requests, world_state, {})

    # Check that the the three role ids are assigned.
    assert len(results) == 3
    assert role_id_a in results
    assert role_id_b in results
    assert role_id_c in results

    # Check that A->0, B->1, C->2, even though A has a higher cost than B for robot 0.
    assert results[role_id_a].role.robot == free_robots[0]
    assert results[role_id_b].role.robot == free_robots[1]
    assert results[role_id_c].role.robot == free_robots[2]

    # Check that the costs for each role result are sqrt(2).
    assert results[role_id_a].cost == math.sqrt(2)
    assert results[role_id_b].cost == math.sqrt(2)
    assert results[role_id_c].cost == math.sqrt(2)


def test_assign_roles_constrained() -> None:
    """Tests that NaiveRoleAssignment.assign_roles respects constraints, ie. even though
    role_id_a and role_id_ball both are HIGH priority, the robot at (0, 0) has the ball
    and thus is assigned BALL_SKILL.

    This test will fail as has_ball has been removed from Robot thus breaking the ball constraint
    """

    # Get the three roles.
    role_id_a, role_id_b, role_id_c, role_id_ball = get_simple_role_ids()

    # Create the cost functions.
    switch_cost = 0.0
    cost_a = cost.distance_to_pt(np.array([0, 0]), math.sqrt(8), switch_cost)
    cost_b = cost.distance_to_pt(np.array([1, 1]), math.sqrt(8), switch_cost)
    cost_c = cost.distance_to_pt(np.array([2, 2]), math.sqrt(8), switch_cost)
    cost_ball = cost.distance_to_pt(np.array([2, 2]), math.sqrt(8), switch_cost)

    # Create the requests in descending priority.
    requests: FlatRoleRequests = {
        role_id_a: role.RoleRequest(Priority.HIGH, required=False, cost_fn=cost_a),
        role_id_ball: role.RoleRequest(
            Priority.HIGH,
            required=False,
            cost_fn=cost_ball,
            constraint_fn=constraint.has_ball(),
        ),
        role_id_b: role.RoleRequest(Priority.MEDIUM, required=False, cost_fn=cost_b),
        role_id_c: role.RoleRequest(Priority.LOW, required=False, cost_fn=cost_c),
    }

    # Create the robots at (0, 0) (1, 1) and (2, 2)
    free_robots = np.array([
        testing.generate_test_robot(robot_id=0,
                                    pose=np.array([0, 0, 0]),
                                    has_ball_sense=True),
        testing.generate_test_robot(robot_id=1, pose=np.array([1, 1, 0])),
        testing.generate_test_robot(robot_id=2, pose=np.array([2, 2, 0])),
    ])

    # Construct the world state.
    out_robots: List[Robot] = list(free_robots)
    their_robots: List[Robot] = []

    world_state: WorldState = testing.generate_test_worldstate(
        our_robots=out_robots, their_robots=their_robots)

    # Assign the roles.
    results = NaiveRoleAssignment.assign_roles(requests, world_state, {})

    # Check that all roles have been assigned.
    assert len(results) == 4
    assert role_id_a in results
    assert role_id_ball in results
    assert role_id_b in results
    assert role_id_c in results

    # Check that A->1, BALL->0, B->2 even though A has a lower cost than BALL for 0.
    assert results[role_id_a].role.robot == free_robots[1]
    assert results[role_id_ball].role.robot == free_robots[0]
    assert results[role_id_b].role.robot == free_robots[2]

    # Check that C's role request is unfilled due to being low priority.
    assert not results[role_id_c].is_filled()
