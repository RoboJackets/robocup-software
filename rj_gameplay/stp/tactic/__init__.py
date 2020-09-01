from typing import Type, TypeVar, List, Optional, Dict, MutableMapping
from abc import ABC, abstractmethod

import stp.skill as skill
import stp.utils.edict as edict
import stp.role as role

SkillT = TypeVar("SkillT", bound=skill.ISkill)


class SkillEntry(edict.EKey[SkillT]):
    """ An entry in the SkillsEnum for a tactic.
    """

    INVALID_IDX = -1

    __slots__ = ["skill", "_idx", "_owner"]

    skill: Optional[SkillT]
    _idx: int

    def __init__(self, entry_skill: Type[SkillT]):
        """
        :param entry_skill: Type of the skill.
        """
        super().__init__(entry_skill)

        self.skill = None
        self._idx = SkillEntry.INVALID_IDX

    def set_idx(self, num: int) -> None:
        self._idx = num

    def set_skill(self, skill_instance: SkillT) -> None:
        self.skill = skill_instance

    def __eq__(self, other) -> bool:
        if not isinstance(other, SkillEntry):
            return False

        return (self.concrete_cls, self._idx) == (other.concrete_cls, other._idx)

    def __hash__(self) -> int:
        return hash((self.concrete_cls, self._idx))

    def __str__(self) -> str:
        return "SkillEntry({:2}: {} - {})".format(
            self._idx, self.concrete_cls.__name__, self.skill
        )

    def __repr__(self) -> str:
        return self.__str__()


class SkillsEnumMeta(type):
    def __new__(mcs, cls, bases, class_dict):
        object_attrs = set(dir(type(cls, (object,), {})))
        enum_cls: type = super().__new__(mcs, cls, bases, class_dict)

        enum_cls.enum_names = [
            key
            for key in class_dict.keys()
            if key not in object_attrs
            and not (key.startswith("_") and key.endswith("_"))
        ]

        return enum_cls


class SkillsEnum(metaclass=SkillsEnumMeta):
    def __init__(self, skill_factory: skill.Factory):
        """
        :param skill_factory: Skill Factory used to initialize the skill instances in
        each SkillEntry.
        """
        # Assign the correct indices to each skill.SkillEntry. Also instantiate the
        # skills using skill_factory.
        for idx, entry in enumerate(self.entries()):
            entry.set_idx(idx)
            entry.set_skill(skill_factory.create(entry.concrete_cls))

    @classmethod
    def entries(cls) -> List[SkillEntry]:
        entries = [getattr(cls, enum_name) for enum_name in cls.enum_names]

        for entry in entries:
            assert isinstance(entry, SkillEntry)

        return entries

    def __repr__(self) -> str:
        strs = [str(entry) for entry in self.entries()]

        return "\n".join(strs)


RoleRequests = Dict[SkillEntry, role.RoleRequest]


class SkillsDict(edict.EDict[List[skill.ISkill]]):
    ...


class ITactic(ABC):
    @abstractmethod
    def get_requests(self, prev_skills: SkillsDict) -> RoleRequests:
        ...


class Ctx:
    """ Context for tactics.
    """

    __slots__ = ["skill_factory"]

    skill_factory: skill.Factory

    def __init__(self, skill_factory: skill.Factory):
        self.skill_factory = skill_factory


TacticT = TypeVar("TacticT", bound=ITactic)


class Registry(MutableMapping):
    """ Registry that holds instances of tactics indexed by the type.
    """

    __slots__ = ["_dict"]

    def __init__(self):
        self._dict: Dict[Type[ITactic], ITactic] = {}

    def __getitem__(self, k: Type[TacticT]) -> TacticT:
        # The below can throw a KeyError.
        tactic: ITactic = self._dict[k]

        # Check that the item we got was an instance of the expected type.
        if not isinstance(skill, k):
            raise KeyError("Tactic {} is not an instance of key {}".format(skill, k))

        return tactic

    def __setitem__(self, k: Type[TacticT], v: TacticT) -> None:
        # Check that the item we're setting is an instance of the expected type.
        if not isinstance(v, k):
            raise KeyError("Tactic {} is not an instance of {}".format(v, k))

        self._dict.__setitem__(k, v)

    def __delitem__(self, k: Type[TacticT]) -> None:
        self._dict.__delitem__(k)

    def __len__(self) -> int:
        return self._dict.__len__()

    def __iter__(self):
        return self._dict.__iter__()

    def __contains__(self, item) -> bool:
        return self._dict.__contains__(item)


class Factory:
    """ Factory that creates tactics according to the TacticRegistry passed in.
    """

    __slots__ = ["_registry"]

    _registry: Registry

    def __init__(self, registry: Registry):
        self._registry = registry

    def create(self, tactic: Type[TacticT]) -> TacticT:
        if tactic not in self._registry:
            # TODO: Create new class for this error category.
            raise ValueError(
                "Trying to create tactic {}, but not in registry!".format(skill)
            )

        return self._registry[tactic]
