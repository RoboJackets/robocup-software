from typing import TypeVar, Generic, Dict, Type
from collections.abc import MutableMapping

ValueInterfaceT = TypeVar("ValueInterfaceT")

ValueConcreteT = TypeVar("ValueConcreteT")


class TypedKey(Generic[ValueConcreteT]):
    __slots__ = ["concrete_cls"]

    def __init__(self, cls: Type[ValueConcreteT]):
        self.concrete_cls = cls

    def value_t(self) -> Type[ValueConcreteT]:
        return self.concrete_cls


class TypedKeyDict(Generic[ValueInterfaceT], MutableMapping):
    """A dictionary with type information where the key encodes information about the
    subclass so that the correct subclass can be retrieved without an additional
    instanceof check.
    """

    __slots__ = ["_dict"]

    def __init__(self):
        self._dict: Dict[TypedKey, ValueInterfaceT] = {}

    def __getitem__(self, k: TypedKey[ValueConcreteT]) -> ValueConcreteT:
        # The below can throw a KeyError.
        item: ValueInterfaceT = self._dict[k]

        # Check that the item we got was an instance of the expected type.
        if not isinstance(item, k.concrete_cls):
            raise KeyError(
                "Item {} from key {} is not an instance of {}".format(
                    item, k, k.concrete_cls
                )
            )

        return item

    def __setitem__(self, k: TypedKey[ValueConcreteT], v: ValueConcreteT) -> None:
        # Check that the item we're setting is an instance of the expected type.
        if not isinstance(v, k.concrete_cls):
            raise KeyError(
                "Item {} from key {} is not an instance of {}".format(
                    v, k, k.concrete_cls
                )
            )

        self._dict.__setitem__(k, v)

    def __delitem__(self, k: TypedKey[ValueConcreteT]) -> None:
        self._dict.__delitem__(k)

    def __len__(self) -> int:
        return self._dict.__len__()

    def __iter__(self):
        return self._dict.__iter__()

    def __contains__(self, item) -> bool:
        return self._dict.__contains__(item)
