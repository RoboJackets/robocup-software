# generated from rosidl_generator_py/resource/_idl.py.em
# with input from rj_msgs:srv/SetFieldDimensions.idl
# generated code does not contain a copyright notice


# Import statements for member types

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_SetFieldDimensions_Request(type):
    """Metaclass of message 'SetFieldDimensions_Request'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('rj_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'rj_msgs.srv.SetFieldDimensions_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__set_field_dimensions__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__set_field_dimensions__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__set_field_dimensions__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__set_field_dimensions__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__set_field_dimensions__request

            from rj_msgs.msg import FieldDimensions
            if FieldDimensions.__class__._TYPE_SUPPORT is None:
                FieldDimensions.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class SetFieldDimensions_Request(metaclass=Metaclass_SetFieldDimensions_Request):
    """Message class 'SetFieldDimensions_Request'."""

    __slots__ = [
        '_field_dimensions',
    ]

    _fields_and_field_types = {
        'field_dimensions': 'rj_msgs/FieldDimensions',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['rj_msgs', 'msg'], 'FieldDimensions'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from rj_msgs.msg import FieldDimensions
        self.field_dimensions = kwargs.get('field_dimensions', FieldDimensions())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.field_dimensions != other.field_dimensions:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @property
    def field_dimensions(self):
        """Message field 'field_dimensions'."""
        return self._field_dimensions

    @field_dimensions.setter
    def field_dimensions(self, value):
        if __debug__:
            from rj_msgs.msg import FieldDimensions
            assert \
                isinstance(value, FieldDimensions), \
                "The 'field_dimensions' field must be a sub message of type 'FieldDimensions'"
        self._field_dimensions = value


# Import statements for member types

# already imported above
# import rosidl_parser.definition


class Metaclass_SetFieldDimensions_Response(type):
    """Metaclass of message 'SetFieldDimensions_Response'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('rj_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'rj_msgs.srv.SetFieldDimensions_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__set_field_dimensions__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__set_field_dimensions__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__set_field_dimensions__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__set_field_dimensions__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__set_field_dimensions__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class SetFieldDimensions_Response(metaclass=Metaclass_SetFieldDimensions_Response):
    """Message class 'SetFieldDimensions_Response'."""

    __slots__ = [
    ]

    _fields_and_field_types = {
    }

    SLOT_TYPES = (
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)


class Metaclass_SetFieldDimensions(type):
    """Metaclass of service 'SetFieldDimensions'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('rj_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'rj_msgs.srv.SetFieldDimensions')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__set_field_dimensions

            from rj_msgs.srv import _set_field_dimensions
            if _set_field_dimensions.Metaclass_SetFieldDimensions_Request._TYPE_SUPPORT is None:
                _set_field_dimensions.Metaclass_SetFieldDimensions_Request.__import_type_support__()
            if _set_field_dimensions.Metaclass_SetFieldDimensions_Response._TYPE_SUPPORT is None:
                _set_field_dimensions.Metaclass_SetFieldDimensions_Response.__import_type_support__()


class SetFieldDimensions(metaclass=Metaclass_SetFieldDimensions):
    from rj_msgs.srv._set_field_dimensions import SetFieldDimensions_Request as Request
    from rj_msgs.srv._set_field_dimensions import SetFieldDimensions_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
