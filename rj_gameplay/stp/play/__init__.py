from abc import ABC, abstractmethod
from typing import Dict, Type, TypeVar, Optional, List, Iterator

import stp.role.assignment as assignment
import stp.tactic as tactic
import stp.utils.typed_key_dict as tkdict
import stp.utils.enum as enum
from stp.role import RoleRequest
from stp.tactic import SkillEntry, ITactic


class IPlay(ABC):
    """Interface for a play, the highest level of abstraction from the STP hierarchy."""

    ...


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
        self._idx = num

    def set_tactic(self, tactic_instance: TacticT) -> None:
        self.tactic = tactic_instance

    def __eq__(self, other) -> bool:
        if not isinstance(other, TacticEntry):
            return False

        return (self.concrete_cls, self._idx) == (other.concrete_cls, other._idx)

    def __hash__(self) -> int:
        return hash((self.concrete_cls, self._idx))

    def __str__(self) -> str:
        return "{:3}: {} - {}".format(self._idx, self.concrete_cls.__name__, self.skill)

    def __repr__(self) -> str:
        return self.__str__()


class TacticsEnum(metaclass=enum.SimpleEnumMeta):
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

    __slots__ = ["tactic_factory"]

    tactic_factory: tactic.Factory

    def __init__(self, tactic_factory: tactic.Factory):
        self.tactic_factory = tactic_factory


RoleRequests = Dict[Type[tactic.ITactic], tactic.RoleRequests]


def flatten_requests(role_requests: RoleRequests) -> assignment.FlatRoleRequests:
    """Flattens play.RoleRequests into assignment.FlatRoleRequests, ie. a nested
    dict into just a flat dict.
    :param role_requests: The nested play.RoleRequests dicts.
    :return: The flattened assignment.FlatRoleRequests dict.
    """
    flat_role_requests: assignment.FlatRoleRequests = {}

    tactic_t: Type[ITactic]
    tactic_requests: Dict[SkillEntry, RoleRequest]

    for tactic_t, tactic_requests in role_requests.items():
        skill_entry: SkillEntry
        request: RoleRequest

        for skill_entry, request in tactic_requests.items():
            flat_role_requests[(tactic_t, skill_entry)] = request

    return flat_role_requests
