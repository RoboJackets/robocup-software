from abc import ABC, abstractmethod
from typing import Type, TypeVar, MutableMapping, Dict

import sheen.role as role


class ISkill(ABC):
    @abstractmethod
    def define(self):
        ...

    @abstractmethod
    def create_request(self) -> role.RoleRequest:
        ...


SkillT = TypeVar("SkillT", bound=ISkill)


class Registry(MutableMapping):
    """ Registry that holds instances of skills indexed by the type.
    """

    __slots__ = ["_dict"]

    def __init__(self):
        self._dict: Dict[Type[ISkill], ISkill] = {}

    def __getitem__(self, k: Type[SkillT]) -> SkillT:
        # The below can throw a KeyError.
        skill: ISkill = self._dict[k]

        # Check that the item we got was an instance of the expected type.
        if not isinstance(skill, k):
            raise KeyError("Skill {} is not an instance of key {}".format(skill, k))

        return skill

    def __setitem__(self, k: Type[SkillT], v: SkillT) -> None:
        # Check that the item we're setting is an instance of the expected type.
        if not isinstance(v, k):
            raise KeyError("Skill {} is not an instance of {}".format(v, k))

        self._dict.__setitem__(k, v)

    def __delitem__(self, k: Type[SkillT]) -> None:
        self._dict.__delitem__(k)

    def __len__(self) -> int:
        return self._dict.__len__()

    def __iter__(self):
        return self._dict.__iter__()

    def __contains__(self, item) -> bool:
        return self._dict.__contains__(item)


class Factory:
    """ Factory that creates skills according to the SkillRegistry passed in.
    """

    __slots__ = ["_registry"]

    _registry: Registry

    def __init__(self, registry: Registry):
        self._registry = registry

    def create(self, skill: Type[SkillT]) -> SkillT:
        if skill not in self._registry:
            # TODO: Create new class for this error category.
            raise ValueError(
                "Trying to create skill {}, but not in registry!".format(skill)
            )

        return self._registry[skill]
