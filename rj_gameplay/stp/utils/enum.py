class SimpleEnumMeta(type):
    """Metaclass for TacticsEnum. Collects all the class attributes and puts it
    in enum_names"""

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
