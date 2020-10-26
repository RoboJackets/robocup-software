"""This module contains the implementation of a metaclass for defining Enums."""

from typing import Any, Dict, List, Tuple, Type, cast


class SimpleEnumMeta(type):
    """Metaclass for TacticsEnum. Collects all the class attributes and puts it
    in enum_names"""

    enum_names: List[str]

    def __new__(
        cls: Type["SimpleEnumMeta"],
        name: str,
        bases: Tuple[type, ...],
        class_dict: Dict[str, Any],
    ) -> "SimpleEnumMeta":
        object_attrs = set(dir(type(name, (object,), {})))
        enum_cls = super().__new__(cls, name, bases, class_dict)

        enum_cls.enum_names = [  # type: ignore
            key
            for key in class_dict.keys()
            if key not in object_attrs
            and not (key.startswith("_") and key.endswith("_"))
        ]

        return cast(SimpleEnumMeta, enum_cls)
