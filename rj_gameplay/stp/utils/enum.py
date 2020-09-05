"""This module contains the implementation of a metaclass for defining Enums."""

from typing import Any, Dict, Tuple


class SimpleEnumMeta(type):
    """Metaclass for TacticsEnum. Collects all the class attributes and puts it
    in enum_names"""

    def __new__(cls: type, name: str, bases: Tuple[Any], class_dict: Dict[str, Any]):
        object_attrs = set(dir(type(name, (object,), {})))
        enum_cls: type = super().__new__(cls, name, bases, class_dict)

        enum_cls.enum_names = [
            key
            for key in class_dict.keys()
            if key not in object_attrs
            and not (key.startswith("_") and key.endswith("_"))
        ]

        return enum_cls
