""" This module contains data structures for the Plays level of STP.
"""

from abc import ABC, abstractmethod
from collections import defaultdict
from typing import Dict, Type, TypeVar, Optional, List, Iterator, Tuple

import stp.role as role
import stp.role.assignment as assignment
import stp.action as action
import stp.tactic as tactic
import stp.utils.typed_key_dict as tkdict
import stp.utils.enum as enum
import stp.rc as rc


TacticT = TypeVar("TacticT", bound=tactic.ITactic)


class TacticEntry(tkdict.TypedKey[TacticT]):
    """An entry in the TacticEnum for a play."""

    __slots__ = ["_idx", "skill"]

    tactic: Optional[TacticT]
    _idx: Optional[int]

    def __init__(self, entry_skill: Type[TacticT]):
        super().__init__(entry_skill)

        self.tactic = None
        self._idx = None

    def set_idx(self, num: int) -> None:
        """Sets the index of the entry."""
        self._idx = num

    def set_tactic(self, tactic_instance: TacticT) -> None:
        """Sets the actual instance of the tactic."""
        self.tactic = tactic_instance

    def __eq__(self, other) -> bool:
        if not isinstance(other, TacticEntry):
            return False

        return (self.concrete_cls, self._idx) == (other.concrete_cls, other._idx)

    def __hash__(self) -> int:
        return hash((self.concrete_cls, self._idx))

    def __str__(self) -> str:
        return "{:3}: {} - {}".format(
            self._idx, self.concrete_cls.__name__, self.tactic
        )

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
            entry.set_tactic(tactic_factory.create(entry.concrete_cls))

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


class IPlay(ABC):
    """Interface for a play, the highest level of abstraction from the STP hierarchy."""

    __slots__ = []

    @abstractmethod
    def tick(
        self, world_state: rc.WorldState, prev_results: assignment.FlatRoleResults
    ) -> Tuple[assignment.FlatRoleResults, List[action.IAction]]:
        """Performs one "tick" of the specified play.

        This should:
            1. Collect all the role requests from the tactics
            2. Perform role assignment
            3. Gives each tactic its assigned roles and getting a list of skills.
            4. Return the list of skills obtained.
        :return: The list of skill to run.
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
    tactic_requests: Dict[tactic.SkillEntry, role.RoleRequest]

    for tactic_t, tactic_requests in role_requests.items():
        skill_entry: tactic.SkillEntry
        request: role.RoleRequest

        for skill_entry, request in tactic_requests.items():
            flat_role_requests[(tactic_t, skill_entry)] = request

    return flat_role_requests


def unflatten_results(results: assignment.FlatRoleResults) -> RoleResults:
    """Unflattens assignment.FlatRoleResults into play.RoleResults, ie. a flat dict
    into a nested dict.
    :param results: The flat assignments.FlatRoleResults dict.
    :return: The nested play.RoleRequests dicts
    """
    nested_results: RoleResults = defaultdict(dict)

    tactic_t: Type[tactic.ITactic]
    skill_entry: tactic.SkillEntry
    for (tactic_t, skill_entry), result in results.items():
        nested_results[tactic_t][skill_entry] = result

    return nested_results
