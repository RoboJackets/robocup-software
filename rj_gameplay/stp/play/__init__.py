""" This module contains data structures for the Plays level of STP.
"""

from abc import ABC, abstractmethod
from collections import defaultdict
from typing import (
    Dict,
    Generic,
    Iterator,
    List,
    Optional,
    Tuple,
    Type,
    TypeVar,
)

import stp.action as action
import stp.skill as skill
import stp.rc as rc
import stp.role as role
import stp.role.assignment as assignment
import stp.tactic as tactic
import stp.utils.enum as enum
import stp.utils.typed_key_dict as tkdict
from stp.role import RoleResult
from stp.tactic import SkillEntry

TacticT = TypeVar("TacticT", bound=tactic.ITactic)


class TacticEntry(tkdict.TypedKey[TacticT]):
    """An entry in the TacticEnum for a play."""

    __slots__ = ["_idx", "_tactic"]

    _tactic: Optional[TacticT]
    _idx: Optional[int]

    def __init__(self, entry_skill: Type[TacticT]):
        super().__init__(entry_skill)

        self._tactic = None
        self._idx = None

    @property
    def tactic(self) -> TacticT:
        """Getter for _tactic that checks that it's not None."""
        assert self._tactic is not None
        return self._tactic

    @tactic.setter
    def tactic(self, tactic_instance: TacticT) -> None:
        """Sets the actual instance of the tactic."""
        assert tactic_instance is not None
        self._tactic = tactic_instance

    def set_idx(self, num: int) -> None:
        """Sets the index of the entry."""
        self._idx = num

    def __eq__(self, other) -> bool:
        if not isinstance(other, TacticEntry):
            return False

        return (self.concrete_cls, self._idx) == (other.concrete_cls,
                                                  other._idx)

    def __hash__(self) -> int:
        return hash((self.concrete_cls, self._idx))

    def __str__(self) -> str:
        return "{:3}: {} - {}".format(self._idx, self.concrete_cls.__name__,
                                      self._tactic)

    def __repr__(self) -> str:
        return self.__str__()


class TacticsEnum(metaclass=enum.SimpleEnumMeta):
    """Enum holding all the tactics that a given play will use."""
    def __init__(self, tactic_factory: tactic.Factory):
        """
        :param tactic_factory: Tactic Factory used to initialize the tactic instances in
        each TacticEntry.
        """
        # Assign the correct indices to each skill.SkillEntry. Also instantiate the
        # skills using skill_factory.
        for idx, entry in enumerate(self.entries()):
            entry.set_idx(idx)
            entry.tactic = tactic_factory.create(entry.concrete_cls)

    @classmethod
    def entries(cls) -> List[TacticEntry]:
        """Returns all the entries of the given tactic"""

        # pylint: disable=no-member
        entries = [getattr(cls, enum_name) for enum_name in cls.enum_names]

        for entry in entries:
            assert isinstance(entry, TacticEntry)

        return entries

    def __iter__(self) -> Iterator[TacticEntry]:
        return self.entries().__iter__()

    def __repr__(self) -> str:
        strs = [str(entry) for entry in self.entries()]

        return "\n".join(strs)


class Ctx:
    """Context for plays."""

    __slots__ = ["tactic_factory", "role_assignment"]

    tactic_factory: tactic.Factory
    role_assignment: assignment.IRoleAssignment

    def __init__(self, tactic_factory: tactic.Factory, role_assignment):
        self.tactic_factory = tactic_factory
        self.role_assignment = role_assignment


PropT = TypeVar("PropT")


class IPlay(Generic[PropT], ABC):
    """Interface for a play, the highest level of abstraction from the STP hierarchy."""

    __slots__ = ()

    @abstractmethod
    def compute_props(self, prev_props: Optional[PropT]) -> PropT:
        """Computes the props(state) required for the current tick.
        :param prev_props: The props from the previous tick, if available.
        :return: The props for the current tick.
        """
        ...

    @abstractmethod
    def tick(
        self,
        world_state: rc.WorldState,
        prev_results: assignment.FlatRoleResults,
        props: PropT,
    ) -> Tuple[Dict[tactic.SkillEntry, List[role.RoleResult]],
               List[tactic.SkillEntry]]:
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
        skill_entry: tactic.SkillEntry
        requests: List[role.RoleRequest]

        for skill_entry, requests in tactic_requests.items():
            request: role.RoleRequest

            for request_idx, request in enumerate(requests):
                flat_role_requests[(tactic_t, skill_entry,
                                    request_idx)] = request

    return flat_role_requests


MaybeRoleResults = Dict[Type[tactic.ITactic],
                        Dict[SkillEntry, List[Optional[role.RoleResult]]]]


def unflatten_results(results: assignment.FlatRoleResults) -> RoleResults:
    """Unflattens assignment.FlatRoleResults into play.RoleResults, ie. a flat dict
    into a nested dict.
    :param results: The flat assignments.FlatRoleResults dict.
    :return: The nested play.RoleRequests dicts
    """
    nested_results: MaybeRoleResults = defaultdict(lambda: defaultdict(list))

    tactic_t: Type[tactic.ITactic]
    skill_entry: tactic.SkillEntry
    for (tactic_t, skill_entry, request_idx), result in results.items():
        results_list: List[
            Optional[RoleResult]] = nested_results[tactic_t][skill_entry]

        # Extend the list so that it's long enough to put in result at request_idx.
        if len(results_list) <= request_idx:
            num_to_extend = request_idx - len(results_list) + 1
            results_list.extend([None for _ in range(num_to_extend)])
        results_list[request_idx] = result

    # Check that there aren't any Nones in the nested dict.
    tactic_results: Dict[SkillEntry, List[Optional[RoleResult]]]
    for tactic_t, tactic_results in nested_results.items():
        for skill_entry, skill_results in tactic_results.items():
            if None in skill_results:
                raise RuntimeError(
                    "Somehow there's a None in the list of RoleResults, meaning that "
                    "we dropped an index somewhere...")

    # mypy fails to infer that there won't be any Nones in the list.
    return nested_results  # type: ignore
