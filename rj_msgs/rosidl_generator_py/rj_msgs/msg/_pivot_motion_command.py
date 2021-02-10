# generated from rosidl_generator_py/resource/_idl.py.em
# with input from rj_msgs:msg/PivotMotionCommand.idl
# generated code does not contain a copyright notice


# Import statements for member types

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_PivotMotionCommand(type):
    """Metaclass of message 'PivotMotionCommand'."""

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
                'rj_msgs.msg.PivotMotionCommand')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__pivot_motion_command
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__pivot_motion_command
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__pivot_motion_command
            cls._TYPE_SUPPORT = module.type_support_msg__msg__pivot_motion_command
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__pivot_motion_command

            from rj_geometry_msgs.msg import Point
            if Point.__class__._TYPE_SUPPORT is None:
                Point.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class PivotMotionCommand(metaclass=Metaclass_PivotMotionCommand):
    """Message class 'PivotMotionCommand'."""

    __slots__ = [
        '_pivot_point',
        '_pivot_target',
    ]

    _fields_and_field_types = {
        'pivot_point': 'rj_geometry_msgs/Point',
        'pivot_target': 'rj_geometry_msgs/Point',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['rj_geometry_msgs', 'msg'], 'Point'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['rj_geometry_msgs', 'msg'], 'Point'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from rj_geometry_msgs.msg import Point
        self.pivot_point = kwargs.get('pivot_point', Point())
        from rj_geometry_msgs.msg import Point
        self.pivot_target = kwargs.get('pivot_target', Point())

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
        if self.pivot_point != other.pivot_point:
            return False
        if self.pivot_target != other.pivot_target:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @property
    def pivot_point(self):
        """Message field 'pivot_point'."""
        return self._pivot_point

    @pivot_point.setter
    def pivot_point(self, value):
        if __debug__:
            from rj_geometry_msgs.msg import Point
            assert \
                isinstance(value, Point), \
                "The 'pivot_point' field must be a sub message of type 'Point'"
        self._pivot_point = value

    @property
    def pivot_target(self):
        """Message field 'pivot_target'."""
        return self._pivot_target

    @pivot_target.setter
    def pivot_target(self, value):
        if __debug__:
            from rj_geometry_msgs.msg import Point
            assert \
                isinstance(value, Point), \
                "The 'pivot_target' field must be a sub message of type 'Point'"
        self._pivot_target = value
