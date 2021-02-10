# generated from rosidl_generator_py/resource/_idl.py.em
# with input from rj_msgs:msg/PathTargetMotionCommand.idl
# generated code does not contain a copyright notice


# Import statements for member types

# Member 'override_angle'
import array  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_PathTargetMotionCommand(type):
    """Metaclass of message 'PathTargetMotionCommand'."""

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
                'rj_msgs.msg.PathTargetMotionCommand')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__path_target_motion_command
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__path_target_motion_command
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__path_target_motion_command
            cls._TYPE_SUPPORT = module.type_support_msg__msg__path_target_motion_command
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__path_target_motion_command

            from rj_geometry_msgs.msg import Point
            if Point.__class__._TYPE_SUPPORT is None:
                Point.__class__.__import_type_support__()

            from rj_msgs.msg import LinearMotionInstant
            if LinearMotionInstant.__class__._TYPE_SUPPORT is None:
                LinearMotionInstant.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class PathTargetMotionCommand(metaclass=Metaclass_PathTargetMotionCommand):
    """Message class 'PathTargetMotionCommand'."""

    __slots__ = [
        '_target',
        '_override_angle',
        '_override_face_point',
    ]

    _fields_and_field_types = {
        'target': 'rj_msgs/LinearMotionInstant',
        'override_angle': 'sequence<double, 1>',
        'override_face_point': 'sequence<rj_geometry_msgs/Point, 1>',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['rj_msgs', 'msg'], 'LinearMotionInstant'),  # noqa: E501
        rosidl_parser.definition.BoundedSequence(rosidl_parser.definition.BasicType('double'), 1),  # noqa: E501
        rosidl_parser.definition.BoundedSequence(rosidl_parser.definition.NamespacedType(['rj_geometry_msgs', 'msg'], 'Point'), 1),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from rj_msgs.msg import LinearMotionInstant
        self.target = kwargs.get('target', LinearMotionInstant())
        self.override_angle = array.array('d', kwargs.get('override_angle', []))
        self.override_face_point = kwargs.get('override_face_point', [])

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
        if self.target != other.target:
            return False
        if self.override_angle != other.override_angle:
            return False
        if self.override_face_point != other.override_face_point:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @property
    def target(self):
        """Message field 'target'."""
        return self._target

    @target.setter
    def target(self, value):
        if __debug__:
            from rj_msgs.msg import LinearMotionInstant
            assert \
                isinstance(value, LinearMotionInstant), \
                "The 'target' field must be a sub message of type 'LinearMotionInstant'"
        self._target = value

    @property
    def override_angle(self):
        """Message field 'override_angle'."""
        return self._override_angle

    @override_angle.setter
    def override_angle(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'd', \
                "The 'override_angle' array.array() must have the type code of 'd'"
            assert len(value) <= 1, \
                "The 'override_angle' array.array() must have a size <= 1"
            self._override_angle = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 len(value) <= 1 and
                 all(isinstance(v, float) for v in value) and
                 True), \
                "The 'override_angle' field must be a set or sequence with length <= 1 and each value of type 'float'"
        self._override_angle = array.array('d', value)

    @property
    def override_face_point(self):
        """Message field 'override_face_point'."""
        return self._override_face_point

    @override_face_point.setter
    def override_face_point(self, value):
        if __debug__:
            from rj_geometry_msgs.msg import Point
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 len(value) <= 1 and
                 all(isinstance(v, Point) for v in value) and
                 True), \
                "The 'override_face_point' field must be a set or sequence with length <= 1 and each value of type 'Point'"
        self._override_face_point = value
