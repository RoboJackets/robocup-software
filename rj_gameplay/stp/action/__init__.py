from abc import ABC, abstractmethod
from typing import Type, TypeVar, MutableMapping, Dict


class IAction(ABC):
    """Interface for actions."""

    @abstractmethod
    def tick(self) -> None:
        """Ticks the action."""
        ...


ActionT = TypeVar("ActionT", bound=IAction)


class Registry(MutableMapping):
    """Registry that holds instances of actions indexed by the type."""

    __slots__ = ["_dict"]

    def __init__(self):
        self._dict: Dict[Type[IAction], IAction] = {}

    def __getitem__(self, k: Type[ActionT]) -> ActionT:
        # The below can throw a KeyError.
        action: IAction = self._dict[k]

        # Check that the item we got was an instance of the expected type.
        if not isinstance(action, k):
            raise KeyError("Action {} is not an instance of key {}".format(action, k))

        return action

    def __setitem__(self, k: Type[ActionT], v: ActionT) -> None:
        # Check that the item we're setting is an instance of the expected type.
        if not isinstance(v, k):
            raise KeyError("Action {} is not an instance of {}".format(v, k))

        self._dict.__setitem__(k, v)

    def __delitem__(self, k: Type[ActionT]) -> None:
        self._dict.__delitem__(k)

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
        if action not in self._registry:
            # TODO: Create new class for this error category.
            raise ValueError(
                "Trying to create action {}, but not in registry!".format(action)
            )

        return self._registry[action]
