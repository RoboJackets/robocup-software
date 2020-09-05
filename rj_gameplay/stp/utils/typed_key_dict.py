"""This module contains data structures to define TypedKeyDict."""

from typing import TypeVar, Generic, Dict, Type
from collections.abc import MutableMapping

ValueInterfaceT = TypeVar("ValueInterfaceT")

ValueConcreteT = TypeVar("ValueConcreteT")


class TypedKey(Generic[ValueConcreteT]):
    """A key that contains type information about the value."""

    __slots__ = ["concrete_cls"]

    def __init__(self, cls: Type[ValueConcreteT]):
        self.concrete_cls = cls

    def value_t(self) -> Type[ValueConcreteT]:
        """Returns the type of the value for this key.
        :return: The type of the value for this key.
        """
        return self.concrete_cls


class TypedKeyDict(Generic[ValueInterfaceT], MutableMapping):
    """A dictionary with type information where the key encodes information about the
    subclass so that the correct subclass can be retrieved without an additional
    instanceof check.
    """

    __slots__ = ["_dict"]

    def __init__(self):
        self._dict: Dict[TypedKey, ValueInterfaceT] = {}

    def __getitem__(self, key: TypedKey[ValueConcreteT]) -> ValueConcreteT:
        # The below can throw a KeyError.
        item: ValueInterfaceT = self._dict[key]

        # Check that the item we got was an instance of the expected type.
        if not isinstance(item, key.concrete_cls):
            raise KeyError(
                "Item {} from key {} is not an instance of {}".format(
                    item, key, key.concrete_cls
                )
            )

        return item

    def __setitem__(self, key: TypedKey[ValueConcreteT], value: ValueConcreteT) -> None:
        # Check that the item we're setting is an instance of the expected type.
        if not isinstance(value, key.concrete_cls):
            raise KeyError(
                "Item {} from key {} is not an instance of {}".format(
                    value, key, key.concrete_cls
                )
            )

        self._dict.__setitem__(key, value)

    def __delitem__(self, key: TypedKey[ValueConcreteT]) -> None:
        self._dict.__delitem__(key)

    def __len__(self) -> int:
        return self._dict.__len__()

    def __iter__(self):
        return self._dict.__iter__()

    def __contains__(self, item) -> bool:
        return self._dict.__contains__(item)
