import math
from typing import List, Optional, Callable

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
import stp.rc as rc
from stp.role import Priority
from stp.role.assignment import FlatRoleRequests, RoleId
from stp.role.assignment.naive import NaiveRoleAssignment, SortedRequests

class AssignCostFn(role.CostFn):

    def __call__(
        self,
        robot: rc.Robot,
        prev_result: Optional["RoleResult"],
        world_state: rc.WorldState,
    ) -> float:

        return -1


    def unassigned_cost_fn(
        self,
        prev_result: Optional["RoleResult"],
        world_state: rc.WorldState,
    ) -> float:

        #TODO: Implement real unassigned cost function
        return 9999

class UnassignCostFn(role.CostFn):

    def __call__(
        self,
        robot: rc.Robot,
        prev_result: Optional["RoleResult"],
        world_state: rc.WorldState,
    ) -> float:

        return 9999


    def unassigned_cost_fn(
        self,
        prev_result: Optional["RoleResult"],
        world_state: rc.WorldState,
    ) -> float:

        #TODO: Implement real unassigned cost function
        return -1

class TestCostFn(role.CostFn):

    def __init__(
        self,
        fn: Callable[[rc.Robot, Optional["RoleResult"], rc.WorldState], float]
    ) -> None:

        self.fn = fn

    def __call__(
        self,
        robot: rc.Robot,
        prev_result: Optional["RoleResult"],
        world_state: rc.WorldState,
    ) -> float:

        return self.fn(robot, prev_result, world_state)

    def unassigned_cost_fn(
        self,
        prev_result: Optional["RoleResult"],
        world_state: rc.WorldState) -> float:

        return 9999

class SkillBase(skill.ISkill):
    def define(self):
        pass

    def tick(self):
        pass

    def create_request(self) -> role.RoleRequest:
        assign_cost_fn = AssignCostFn()
        # TODO change priority float to something useful
        return role.RoleRequest(
            3.0, required=True, cost_fn=assign_cost_fn)

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
        # TODO change priority floats to something useful
        role_requests: tactic.RoleRequests = {
            self.A1: [self.A1.skill.create_request().with_priority(3.0)],
            self.A2: [self.A2.skill.create_request().with_priority(2.0)],
            self.B1: [self.B1.skill.create_request().with_priority(2.0)],
            self.B2: [self.B2.skill.create_request().with_priority(1.0)],
            self.C1: [self.C1.skill.create_request().with_priority(3.0)],
            self.C2: [self.C2.skill.create_request().with_priority(2.0)],
            self.BALL_SKILL: [
                self.BALL_SKILL.skill.create_request()
                .with_priority(1.0)
                .with_constraint_fn(constraint.has_ball())
            ],
        }

        return role_requests

    def create_request(self):
        pass


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

    # TODO change priority float to something useful
    requests: FlatRoleRequests = {
        role_id_a: role.RoleRequest(
            1.0, required=True, cost_fn=constant_cost
        ),
        role_id_b: role.RoleRequest(3.0, required=True, cost_fn=constant_cost),
        role_id_c: role.RoleRequest(
            2.0, required=True, cost_fn=constant_cost
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
    # TODO change priority float to something useful
    assert role_id_a in sorted_requests[1.0]
    assert role_id_b in sorted_requests[3.0]
    assert role_id_c in sorted_requests[2.0]

    # Check that each of the role requests are equal.
    # TODO change priority float to something useful
    assert sorted_requests[3.0][role_id_b] == requests[role_id_b]
    assert sorted_requests[2.0][role_id_c] == requests[role_id_c]
    assert sorted_requests[1.0][role_id_a] == requests[role_id_a]


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

    # TODO change priority float to something useful
    assert len(sorted_requests) == 3
    assert len(sorted_requests[3.0]) == len(low_tactics)
    assert len(sorted_requests[2.0]) == len(med_tactics)
    assert len(sorted_requests[1.0]) == len(hi_tactics)

    # TODO change priority float to something useful
    for low_tactic in low_tactics:
        assert (TacticBase, low_tactic, 0) in sorted_requests[3.0]
        assert (
            sorted_requests[3.0][TacticBase, low_tactic, 0]
            == requests[TacticBase][low_tactic][0]
        )

    # TODO change priority float to something useful
    for med_tactic in med_tactics:
        assert (TacticBase, med_tactic, 0) in sorted_requests[2.0]
        assert (
            sorted_requests[2.0][TacticBase, med_tactic, 0]
            == requests[TacticBase][med_tactic][0]
        )

    # TODO change priority float to something useful
    for hi_tactic in hi_tactics:
        assert (TacticBase, hi_tactic, 0) in sorted_requests[1.0]
        assert (
            sorted_requests[1.0][TacticBase, hi_tactic, 0]
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

    # Create CostFns
    cost_fn_a = TestCostFn(cost_a)
    cost_fn_b = TestCostFn(cost_b)
    cost_fn_c = TestCostFn(cost_c)

    # Create the requests of same priority.
    # TODO change priority float to something useful
    requests: FlatRoleRequests = {
<<<<<<< HEAD
        role_id_a: role.RoleRequest(3.0, required=True, cost_fn=cost_a),
        role_id_b: role.RoleRequest(3.0, required=True, cost_fn=cost_b),
        role_id_c: role.RoleRequest(3.0, required=True, cost_fn=cost_c),
=======
        role_id_a: role.RoleRequest(Priority.LOW, required=True, cost_fn=cost_fn_a),
        role_id_b: role.RoleRequest(Priority.LOW, required=True, cost_fn=cost_fn_b),
        role_id_c: role.RoleRequest(Priority.LOW, required=True, cost_fn=cost_fn_c),
>>>>>>> ae2920b8b98213e625d0565dd67005e7a8595fac
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
    assert costs_matrix.shape == (5, 3)

    # fmt: off
    expected_costs_matrix = np.array(
        [[0.0,              math.sqrt(2),       math.sqrt(8)],
         [math.sqrt(2),              0.0,       math.sqrt(2)],
         [math.sqrt(8),     math.sqrt(2),                0.0],
         [math.sqrt(8),     math.sqrt(8),       math.sqrt(2)],
         [9999,                     9999,               9999]]
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

    # Create CostFns
    cost_fn_a = TestCostFn(cost_a)
    cost_fn_b = TestCostFn(cost_b)
    cost_fn_c = TestCostFn(cost_c)

    # Create the requests of same priority.
    # TODO change priority float to something useful
    requests: FlatRoleRequests = {
<<<<<<< HEAD
        role_id_a: role.RoleRequest(3.0, required=True, cost_fn=cost_a),
        role_id_b: role.RoleRequest(3.0, required=True, cost_fn=cost_b),
        role_id_c: role.RoleRequest(3.0, required=True, cost_fn=cost_c),
=======
        role_id_a: role.RoleRequest(Priority.LOW, required=True, cost_fn=cost_fn_a),
        role_id_b: role.RoleRequest(Priority.LOW, required=True, cost_fn=cost_fn_b),
        role_id_c: role.RoleRequest(Priority.LOW, required=True, cost_fn=cost_fn_c),
>>>>>>> ae2920b8b98213e625d0565dd67005e7a8595fac
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

    # Create CosFns
    cost_fn_a = TestCostFn(cost_a)
    cost_fn_b = TestCostFn(cost_b)
    cost_fn_c = TestCostFn(cost_c)

    # Create the requests in descending priority.
    # TODO change priority float to something useful
    requests: FlatRoleRequests = {
<<<<<<< HEAD
        role_id_a: role.RoleRequest(1.0, required=True, cost_fn=cost_a),
        role_id_b: role.RoleRequest(2.0, required=True, cost_fn=cost_b),
        role_id_c: role.RoleRequest(3.0, required=True, cost_fn=cost_c),
=======
        role_id_a: role.RoleRequest(Priority.HIGH, required=True, cost_fn=cost_fn_a),
        role_id_b: role.RoleRequest(Priority.MEDIUM, required=True, cost_fn=cost_fn_b),
        role_id_c: role.RoleRequest(Priority.LOW, required=True, cost_fn=cost_fn_c),
>>>>>>> ae2920b8b98213e625d0565dd67005e7a8595fac
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

    # Create role assigner
    role_assigner = NaiveRoleAssignment()

    # Get the four roles.
    role_id_a, role_id_b, role_id_c, role_id_ball = get_simple_role_ids()

    # Create the cost functions.
    switch_cost = 0.0
    cost_a = cost.distance_to_pt(np.array([0, 0]), math.sqrt(8), switch_cost)
    cost_b = cost.distance_to_pt(np.array([1, 1]), math.sqrt(8), switch_cost)
    cost_c = cost.distance_to_pt(np.array([2, 2]), math.sqrt(8), switch_cost)
    cost_ball = cost.distance_to_pt(np.array([2, 2]), math.sqrt(8), switch_cost)

    # Create CosFns
    cost_fn_a = TestCostFn(cost_a)
    cost_fn_b = TestCostFn(cost_b)
    cost_fn_c = TestCostFn(cost_c)
    cost_fn_ball = TestCostFn(cost_ball)

    # Create the requests in descending priority.
    # TODO change priority float to something useful
    requests: FlatRoleRequests = {
<<<<<<< HEAD
        role_id_a: role.RoleRequest(1.0, required=False, cost_fn=cost_a),
=======
        role_id_a: role.RoleRequest(Priority.HIGH, required=False, cost_fn=cost_fn_a),
>>>>>>> ae2920b8b98213e625d0565dd67005e7a8595fac
        role_id_ball: role.RoleRequest(
            1.0,
            required=False,
            cost_fn=cost_fn_ball,
            constraint_fn=constraint.has_ball(),
        ),
<<<<<<< HEAD
        role_id_b: role.RoleRequest(2.0, required=False, cost_fn=cost_b),
        role_id_c: role.RoleRequest(3.0, required=False, cost_fn=cost_c),
=======
        role_id_b: role.RoleRequest(Priority.MEDIUM, required=False, cost_fn=cost_fn_b),
        role_id_c: role.RoleRequest(Priority.LOW, required=False, cost_fn=cost_fn_c),
>>>>>>> ae2920b8b98213e625d0565dd67005e7a8595fac
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


def test_unassigned_role() -> None:

    # Get the four roles.
    role_id_a, role_id_b, role_id_c, role_id_d = get_simple_role_ids()

    # Create role assigner
    role_assigner = NaiveRoleAssignment()

    assign_cost_fn = AssignCostFn()
    unassigned_cost_fn = UnassignCostFn()
    requests: FlatRoleRequests = {
        role_id_a: role.RoleRequest(Priority.HIGH, required=False, cost_fn=assign_cost_fn),
        role_id_b: role.RoleRequest(
            Priority.HIGH,
            required=False,
            cost_fn=assign_cost_fn,
            constraint_fn=constraint.has_ball(),
        ),
        role_id_c: role.RoleRequest(Priority.HIGH, required=False, cost_fn=assign_cost_fn),
        role_id_d: role.RoleRequest(Priority.HIGH, required=False, cost_fn=unassigned_cost_fn),
    }

    # Create the robots at (0, 0) (1, 1) and (2, 2)
    free_robots = np.array([
        testing.generate_test_robot(robot_id=0,
                                    pose=np.array([0, 0, 0]),
                                    has_ball_sense=True),
        testing.generate_test_robot(robot_id=1, pose=np.array([1, 1, 0])),
        testing.generate_test_robot(robot_id=2, pose=np.array([2, 2, 0])),
        testing.generate_test_robot(robot_id=3, pose=np.array([2, 1, 0])),
        testing.generate_test_robot(robot_id=4, pose=np.array([1, 2, 0]))
    ])

    # Construct the world state.
    out_robots: List[Robot] = list(free_robots)
    their_robots: List[Robot] = []

    world_state: WorldState = testing.generate_test_worldstate(
        our_robots=out_robots, their_robots=their_robots)

    # Assign the roles.
    results = role_assigner.assign_roles(requests, world_state, {})

    # Check that all roles are returned in results.
    assert len(results) == 4
    assert role_id_a in results
    assert role_id_b in results
    assert role_id_c in results
    assert role_id_d in results

    # Check that D's role request is unfilled due to unassigned cost function.
    assert not results[role_id_d].is_filled()

def test_unassigned_roles() -> None:

    # Get the four roles.
    role_id_a, role_id_b, role_id_c, role_id_d = get_simple_role_ids()

    # Create role assigner
    role_assigner = NaiveRoleAssignment()

    # Create cost functions
    assign_cost_fn = AssignCostFn()
    unassigned_cost_fn = UnassignCostFn()

    # Create role requests
    requests: FlatRoleRequests = {
        role_id_a: role.RoleRequest(Priority.HIGH, required=False, cost_fn=unassigned_cost_fn),
        role_id_b: role.RoleRequest(
            Priority.HIGH,
            required=False,
            cost_fn=unassigned_cost_fn,
            constraint_fn=constraint.has_ball(),
        ),
        role_id_c: role.RoleRequest(Priority.HIGH, required=False, cost_fn=unassigned_cost_fn),
        role_id_d: role.RoleRequest(Priority.HIGH, required=False, cost_fn=unassigned_cost_fn),
    }

    # Create the robots at (0, 0) (1, 1) and (2, 2)
    free_robots = np.array([
        testing.generate_test_robot(robot_id=0,
                                    pose=np.array([0, 0, 0]),
                                    has_ball_sense=True),
        testing.generate_test_robot(robot_id=1, pose=np.array([1, 1, 0])),
        testing.generate_test_robot(robot_id=2, pose=np.array([2, 2, 0])),
        testing.generate_test_robot(robot_id=3, pose=np.array([2, 1, 0])),
        testing.generate_test_robot(robot_id=4, pose=np.array([1, 2, 0]))
    ])

    # Construct the world state.
    out_robots: List[Robot] = list(free_robots)
    their_robots: List[Robot] = []

    world_state: WorldState = testing.generate_test_worldstate(
        our_robots=out_robots, their_robots=their_robots)

    # Assign the roles.
    results = role_assigner.assign_roles(requests, world_state, {})

    # Check that all roles are returned in results.
    assert len(results) == 4
    assert role_id_a in results
    assert role_id_b in results
    assert role_id_c in results
    assert role_id_d in results

    # Check that no role requests are filled due to the cost function.
    assert not results[role_id_a].is_filled()
    assert not results[role_id_b].is_filled()
    assert not results[role_id_c].is_filled()
    assert not results[role_id_d].is_filled()
