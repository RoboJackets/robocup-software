"""This module contains Actions of the STP(A) hierarchy."""

from abc import ABC, abstractmethod
from typing import Dict, MutableMapping, Type, TypeVar
from rclpy.publisher import publisher


class IAction(ABC):
    """Interface for actions."""

    #@abstractmethod
    def tick(self) -> None:
        """Ticks the action. Publishing RobotIntent on the provided publisher if applicable"""
        ...

    #@abstractmethod
    #def tick(self) -> RobotIntent:
    #    """Ticks the action, returns a RobotIntent to forward the action"""
    #    ...

    #@abstractmethod
    #def tick(self) -> RobotIntent:
    #    """Ticks the action, returns an object with only relavant fields of the RobotIntent message"""
    #    ...
    #"""


class IFiniteAction(IAction, ABC):
    """Represents actions that have a start and a finish
        
       It's likly that this won't be needed once we have a proper action implementation.
    """
    
    @abstractmethod
    def is_done(self, world_state: rc.WorldState) -> bool:
        """
        Function that detects if an action is complete by looking at the world state.
        We don't want to have to do this and we won't once there is bidirectional communication between gameplay and robot-driving systems, which should happen with a proper action implementation.
        """
        ...

    @abstractmethod
    def finish(self) -> rj_msgs.msgs.RobotIntent:
        """
        Returns a robot-intent that stops the current function???
        """
        ...


ActionT = TypeVar("ActionT", bound=IAction)


class Registry:
    """Registry that holds instances of actions indexed by the type."""

    __slots__ = ["_dict"]

    def __init__(self):
        self._dict: Dict[Type[IAction], IAction] = {}

    def __getitem__(self, key: Type[ActionT]) -> ActionT:
        # The below can throw a KeyError.
        action: IAction = self._dict[key]

        # Check that the item we got was an instance of the expected type.
        if not isinstance(action, key):
            raise KeyError("Action {} is not an instance of key {}".format(action, key))

        return action

    def __setitem__(self, key: Type[ActionT], value: ActionT) -> None:
        # Check that the item we're setting is an instance of the expected type.
        if not isinstance(value, key):
            raise KeyError("Action {} is not an instance of {}".format(value, key))

        self._dict.__setitem__(key, value)

    def __delitem__(self, key: Type[ActionT]) -> None:
        self._dict.__delitem__(key)

    def __len__(self) -> int:
        return self._dict.__len__()

    def __iter__(self):
        return self._dict.__iter__()

    def __contains__(self, item) -> bool:
        return self._dict.__contains__(item)


class Factory:
    """Factory that creates actions according to the ActionRegistry passed in."""

    __slots__ = ["_registry"]

    _registry: Registry

    def __init__(self, registry: Registry):
        self._registry = registry

    def create(self, action: Type[ActionT]) -> ActionT:
        """Creates an instance of the action given the type of the interface of the
        action."""
        if action not in self._registry:
            # TODO: Create new class for this error category.
            raise ValueError(
                "Trying to create action {}, but not in registry!".format(action)
            )

        return self._registry[action]
