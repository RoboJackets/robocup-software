""" This module contains data structures for the Plays level of STP.
"""

from abc import ABC, abstractmethod
from collections import defaultdict
from typing import Dict, Generic, List, Optional, Tuple, Type, TypeVar

import stp.action as action
import stp.skill as skill
import stp.rc as rc
import stp.role as role
import stp.role.assignment as assignment
import stp.tactic as tactic
import stp.utils.enum as enum
from stp.role import RoleResult

class Ctx:
    """Context for plays."""

    __slots__ = ["role_assignment"]

    role_assignment: assignment.IRoleAssignment

    def __init__(self, role_assignment):
        self.role_assignment = role_assignment


PropT = TypeVar("PropT")


class IPlay(Generic[PropT], ABC):
    """Interface for a play, the highest level of abstraction from the STP hierarchy."""

    __slots__ = ()

    @abstractmethod
    def tick(
        self,
        world_state: rc.WorldState,
        prev_results: assignment.FlatRoleResults,
        props: PropT,
    ) -> Tuple[Dict[skill.ISkill, role.RoleRequest], PropT]:
        """Performs one "tick" of the specified play.

        This should:
            1. Collect all the role requests from the tactics
            2. Perform role assignment
            3. Gives each tactic its assigned roles and getting a list of skills.
            4. Return the list of skills obtained.
        :param world_state: Current state of the world.
        :param prev_results: Previous results of role assignment.
        :param props: Props from compute_props.
        :return: Tuple of the role and skills.
        """
        ...


RoleRequests = Dict[Type[tactic.ITactic], tactic.RoleRequests]
RoleResults = Dict[Type[tactic.ITactic], tactic.RoleResults]


def flatten_requests(role_requests: RoleRequests) -> assignment.FlatRoleRequests:
    """Flattens play.RoleRequests into assignment.FlatRoleRequests, ie. a nested
    dict into just a flat dict.
    :param role_requests: The nested play.RoleRequests dicts.
    :return: The flattened assignment.FlatRoleRequests dict.
    """
    flat_role_requests: assignment.FlatRoleRequests = {}

    tactic_t: Type[tactic.ITactic]
    tactic_requests: tactic.RoleRequests

    for tactic_t, tactic_requests in role_requests.items():
        skill: skill.ISkill
        requests: List[role.RoleRequest]

        for skill, requests in tactic_requests.items():
            request: role.RoleRequest

            for request_idx, request in enumerate(requests):
                flat_role_requests[(tactic_t, skill, request_idx)] = request

    return flat_role_requests


MaybeRoleResults = Dict[
    Type[tactic.ITactic], Dict[skill.ISkill, List[Optional[role.RoleResult]]]
]


def unflatten_results(results: assignment.FlatRoleResults) -> RoleResults:
    """Unflattens assignment.FlatRoleResults into play.RoleResults, ie. a flat dict
    into a nested dict.
    :param results: The flat assignments.FlatRoleResults dict.
    :return: The nested play.RoleRequests dicts
    """
    nested_results: MaybeRoleResults = defaultdict(lambda: defaultdict(list))

    tactic_t: Type[tactic.ITactic]
    skill: skill.ISkill
    for (tactic_t, skill, request_idx), result in results.items():
        results_list: List[Optional[RoleResult]] = nested_results[tactic_t][skill]

        # Extend the list so that it's long enough to put in result at request_idx.
        if len(results_list) <= request_idx:
            num_to_extend = request_idx - len(results_list) + 1
            results_list.extend([None for _ in range(num_to_extend)])
        results_list[request_idx] = result

    # Check that there aren't any Nones in the nested dict.
    tactic_results: Dict[SkillEntry, List[Optional[RoleResult]]]
    for tactic_t, tactic_results in nested_results.items():
        for skill, skill_results in tactic_results.items():
            if None in skill_results:
                raise RuntimeError(
                    "Somehow there's a None in the list of RoleResults, meaning that "
                    "we dropped an index somewhere..."
                )

    # mypy fails to infer that there won't be any Nones in the list.
    return nested_results  # type: ignore
