""" This module contains data structures for the Tactics level of STP.
"""

from abc import ABC, abstractmethod
from typing import Type, TypeVar, List, Optional, Dict, MutableMapping, Tuple

import stp.action as action
import stp.skill as skill
import stp.utils.typed_key_dict as tkdict
import stp.utils.enum as enum
import stp.role as role

SkillT = TypeVar("SkillT", bound=skill.ISkill)


class SkillEntry(tkdict.TypedKey[SkillT]):
    """An entry in the SkillsEnum for a tactic."""

    __slots__ = ["skill", "_idx", "_owner"]

    skill: Optional[SkillT]
    _idx: Optional[int]

    def __init__(self, entry_skill: Type[SkillT]):
        """
        :param entry_skill: Type of the skill.
        """
        super().__init__(entry_skill)

        self.skill = None
        self._idx = None

    def set_idx(self, num: int) -> None:
        """Sets the index of the current skill, used for equality checks & hashing."""
        self._idx = num

    def set_skill(self, skill_instance: SkillT) -> None:
        """Sets the instance of the skill."""
        self.skill = skill_instance

    def __eq__(self, other) -> bool:
        if not isinstance(other, SkillEntry):
            return False

        return (self.concrete_cls, self._idx) == (other.concrete_cls, other._idx)

    def __hash__(self) -> int:
        return hash((self.concrete_cls, self._idx))

    def __str__(self) -> str:
        idx_string: str = "{:2}".format(self._idx) if self._idx else "??"
        return "SkillEntry({}: {} - {})".format(
            idx_string, self.concrete_cls.__name__, self.skill
        )

    def __repr__(self) -> str:
        return self.__str__()


class SkillsEnum(metaclass=enum.SimpleEnumMeta):
    """Enum that holds skills."""

    def __init__(self, skill_factory: skill.Factory):
        """Instantiates the SkillsEnum by instantiating it with concrete skills via the
        skill.Factory passed in.
        :param skill_factory: Skill Factory used to initialize the skill instances in
        each SkillEntry.
        """
        for idx, entry in enumerate(self.entries()):
            # Assign the correct indices to each skill.SkillEntry.
            entry.set_idx(idx)
            # Instantiate the skills using skill_factory.
            entry.set_skill(skill_factory.create(entry.concrete_cls))

    @classmethod
    def entries(cls) -> List[SkillEntry]:
        """Returns a list of the SkillEntry for this SkillsEnum.
        :return: List of the SkillEntry for this SkillsEnum.
        """
        # pylint: disable=no-member
        entries = [getattr(cls, enum_name) for enum_name in cls.enum_names]

        for entry in entries:
            assert isinstance(entry, SkillEntry)

        return entries

    def __repr__(self) -> str:
        strs = [str(entry) for entry in self.entries()]

        return "\n".join(strs)


RoleRequests = Dict[SkillEntry, role.RoleRequest]
RoleResults = Dict[SkillEntry, role.RoleResult]


class SkillsDict(tkdict.TypedKeyDict[List[skill.ISkill]]):
    """A dictionary mapping typed keys to a list of skill.ISkill."""

    ...


class ITactic(ABC):
    """The interface class for all tactics."""

    @abstractmethod
    def get_requests(self, prev_skills: SkillsDict) -> RoleRequests:
        """Returns the RoleRequests for this tactic.
        :param prev_skills: A dictionary of skills from the previous iteration.
        :return:
        """
        ...

    @abstractmethod
    def tick(
        self, role_results: RoleResults
    ) -> Tuple[List[action.IAction], SkillsDict]:
        """Ticks the tactic, returning a tuple of the actions and the skills executed.
        :param role_results: The results of role assignment.
        :return:
        """
        ...


class Ctx:
    """Context for tactics."""

    __slots__ = ["skill_factory"]

    skill_factory: skill.Factory

    def __init__(self, skill_factory: skill.Factory):
        self.skill_factory = skill_factory


TacticT = TypeVar("TacticT", bound=ITactic)


class Registry:
    """Registry that holds instances of tactics indexed by the type."""

    __slots__ = ["_dict"]

    def __init__(self):
        self._dict: Dict[Type[ITactic], ITactic] = {}

    def __getitem__(self, key: Type[TacticT]) -> TacticT:
        # The below can throw a KeyError.
        tactic: ITactic = self._dict[key]

        # Check that the item we got was an instance of the expected type.
        if not isinstance(skill, key):
            raise KeyError("Tactic {} is not an instance of key {}".format(skill, key))

        return tactic

    def __setitem__(self, key: Type[TacticT], value: TacticT) -> None:
        # Check that the item we're setting is an instance of the expected type.
        if not isinstance(value, key):
            raise KeyError("Tactic {} is not an instance of {}".format(value, key))

        self._dict.__setitem__(key, value)

    def __delitem__(self, key: Type[TacticT]) -> None:
        self._dict.__delitem__(key)

    def __len__(self) -> int:
        return self._dict.__len__()

    def __iter__(self):
        return self._dict.__iter__()

    def __contains__(self, item) -> bool:
        return self._dict.__contains__(item)


class Factory:
    """Factory that creates tactics according to the TacticRegistry passed in."""

    __slots__ = ["_registry"]

    _registry: Registry

    def __init__(self, registry: Registry):
        self._registry = registry

    def create(self, tactic: Type[TacticT]) -> TacticT:
        """Creates an instance of the tactic given the type of the interface of the
        tactic."""
        if tactic not in self._registry:
            # TODO: Create new class for this error category.
            raise ValueError(
                "Trying to create tactic {}, but not in registry!".format(skill)
            )

        return self._registry[tactic]
