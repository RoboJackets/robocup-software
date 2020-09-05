from typing import List, Optional

import numpy as np

import stp.role.cost as cost
import stp.role as role
import stp.skill as skill
import stp.tactic as tactic
import stp.play as play
from stp import action as action

from stp.role import Priority
from stp.rc import Ball, Robot, WorldState
from stp.role.assignment import RoleId, FlatRoleRequests
from stp.tactic import RoleResults, PropT


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


class Skills(tactic.SkillsEnum):
    A1 = tactic.SkillEntry(SkillA)
    A2 = tactic.SkillEntry(SkillA)
    B1 = tactic.SkillEntry(SkillB)
    B2 = tactic.SkillEntry(SkillB)
    C1 = tactic.SkillEntry(SkillC)
    C2 = tactic.SkillEntry(SkillC)


class TacticBase(tactic.ITactic[None]):
    def __init__(self, ctx: tactic.Ctx):
        self.skills = Skills(ctx.skill_factory)

        self.A1 = self.skills.A1
        self.A2 = self.skills.A2
        self.B1 = self.skills.B1
        self.B2 = self.skills.B2
        self.C1 = self.skills.C1
        self.C2 = self.skills.C2

    def compute_props(self, prev_props: None) -> None:
        return None

    def tick(self, role_results: RoleResults, props: None) -> List[action.IAction]:
        # Dummy tick function doesn't return any actions.
        return []

    def get_requests(self, world_state: WorldState, props: None) -> tactic.RoleRequests:
        role_requests: tactic.RoleRequests = {
            self.A1: [self.A1.skill.create_request()],
            self.A2: self.A2.skill.create_requests(5),
            self.B1: self.B1.skill.create_requests(3),
            self.B2: [self.B2.skill.create_request()],
            self.C1: [self.C1.skill.create_request()],
            self.C2: [self.C2.skill.create_request()],
        }

        return role_requests


def get_tactic_ctx() -> tactic.Ctx:
    """Creates a simple tactic context for convenience.
    :return: Tactic context containing SkillA, SkillB and SkillC.
    """
    skill_registry = skill.Registry()

    skill_registry[SkillA] = SkillA()
    skill_registry[SkillB] = SkillB()
    skill_registry[SkillC] = SkillC()

    skill_factory = skill.Factory(skill_registry)
    return tactic.Ctx(skill_factory)


def test_flatten_requests() -> None:
    """Tests that play.flatten_requests works as expected."""
    tactic_ctx = get_tactic_ctx()
    tactic_instance = TacticBase(tactic_ctx)

    # Create dummy world_state.
    out_robots: List[Robot] = []
    their_robots: List[Robot] = []
    ball: Ball = Ball(np.zeros(2), np.zeros(2))

    world_state: WorldState = WorldState(out_robots, their_robots, ball)

    requests: play.RoleRequests = {
        TacticBase: tactic_instance.get_requests(world_state, None)
    }

    flat_requests: FlatRoleRequests = play.flatten_requests(requests)

    # Check that the flattened requests has the correct length.
    assert len(flat_requests) == 1 + 5 + 3 + 1 + 1 + 1

    # Check that each item of the dictionary is what we expecte it to be.
    for skill_entry in [
        tactic_instance.A1,
        tactic_instance.A2,
        tactic_instance.B1,
        tactic_instance.B2,
        tactic_instance.C1,
        tactic_instance.C2,
    ]:
        for request_idx, request in enumerate(requests[TacticBase][skill_entry]):
            assert request == flat_requests[TacticBase, skill_entry, request_idx]
