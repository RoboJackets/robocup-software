""" This module contains data structures for the Skills level of STP.
"""

from abc import ABC, abstractmethod
from typing import Type, TypeVar, MutableMapping, Dict

import stp.role as role


class ISkill(ABC):
    """ Interface for Skills. """

    @abstractmethod
    def define(self):
        """Defines the skill. TODO: Flesh this out."""
        ...

    @abstractmethod
    def create_request(self) -> role.RoleRequest:
        """Creates a sane default RoleRequest."""
        ...


SkillT = TypeVar("SkillT", bound=ISkill)


class Registry:
    """Registry that holds instances of skills indexed by the type."""

    __slots__ = ["_dict"]

    def __init__(self):
        self._dict: Dict[Type[ISkill], ISkill] = {}

    def __getitem__(self, key: Type[SkillT]) -> SkillT:
        # The below can throw a KeyError.
        skill: ISkill = self._dict[key]

        # Check that the item we got was an instance of the expected type.
        if not isinstance(skill, key):
            raise KeyError("Skill {} is not an instance of key {}".format(skill, key))

        return skill

    def __setitem__(self, key: Type[SkillT], value: SkillT) -> None:
        # Check that the item we're setting is an instance of the expected type.
        if not isinstance(value, key):
            raise KeyError("Skill {} is not an instance of {}".format(value, key))

        self._dict.__setitem__(key, value)

    def __delitem__(self, key: Type[SkillT]) -> None:
        self._dict.__delitem__(key)

    def __len__(self) -> int:
        return self._dict.__len__()

    def __iter__(self):
        return self._dict.__iter__()

    def __contains__(self, item) -> bool:
        return self._dict.__contains__(item)


class Factory:
    """Factory that creates skills according to the SkillRegistry passed in."""

    __slots__ = ["_registry"]

    _registry: Registry

    def __init__(self, registry: Registry):
        self._registry = registry

    def create(self, skill: Type[SkillT]) -> SkillT:
        """Creates an instance of the skill given the type of the interface of the
        skill."""
        if skill not in self._registry:
            # TODO: Create new class for this error category.
            raise ValueError(
                "Trying to create skill {}, but not in registry!".format(skill)
            )

        return self._registry[skill]
