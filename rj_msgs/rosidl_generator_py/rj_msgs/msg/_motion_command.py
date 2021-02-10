# generated from rosidl_generator_py/resource/_idl.py.em
# with input from rj_msgs:msg/MotionCommand.idl
# generated code does not contain a copyright notice


# Import statements for member types

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_MotionCommand(type):
    """Metaclass of message 'MotionCommand'."""

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
                'rj_msgs.msg.MotionCommand')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__motion_command
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__motion_command
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__motion_command
            cls._TYPE_SUPPORT = module.type_support_msg__msg__motion_command
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__motion_command

            from rj_msgs.msg import CollectMotionCommand
            if CollectMotionCommand.__class__._TYPE_SUPPORT is None:
                CollectMotionCommand.__class__.__import_type_support__()

            from rj_msgs.msg import EmptyMotionCommand
            if EmptyMotionCommand.__class__._TYPE_SUPPORT is None:
                EmptyMotionCommand.__class__.__import_type_support__()

            from rj_msgs.msg import InterceptMotionCommand
            if InterceptMotionCommand.__class__._TYPE_SUPPORT is None:
                InterceptMotionCommand.__class__.__import_type_support__()

            from rj_msgs.msg import LineKickMotionCommand
            if LineKickMotionCommand.__class__._TYPE_SUPPORT is None:
                LineKickMotionCommand.__class__.__import_type_support__()

            from rj_msgs.msg import PathTargetMotionCommand
            if PathTargetMotionCommand.__class__._TYPE_SUPPORT is None:
                PathTargetMotionCommand.__class__.__import_type_support__()

            from rj_msgs.msg import PivotMotionCommand
            if PivotMotionCommand.__class__._TYPE_SUPPORT is None:
                PivotMotionCommand.__class__.__import_type_support__()

            from rj_msgs.msg import SettleMotionCommand
            if SettleMotionCommand.__class__._TYPE_SUPPORT is None:
                SettleMotionCommand.__class__.__import_type_support__()

            from rj_msgs.msg import WorldVelMotionCommand
            if WorldVelMotionCommand.__class__._TYPE_SUPPORT is None:
                WorldVelMotionCommand.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class MotionCommand(metaclass=Metaclass_MotionCommand):
    """Message class 'MotionCommand'."""

    __slots__ = [
        '_empty_command',
        '_path_target_command',
        '_world_vel_command',
        '_pivot_command',
        '_settle_command',
        '_collect_command',
        '_line_kick_command',
        '_intercept_command',
    ]

    _fields_and_field_types = {
        'empty_command': 'sequence<rj_msgs/EmptyMotionCommand, 1>',
        'path_target_command': 'sequence<rj_msgs/PathTargetMotionCommand, 1>',
        'world_vel_command': 'sequence<rj_msgs/WorldVelMotionCommand, 1>',
        'pivot_command': 'sequence<rj_msgs/PivotMotionCommand, 1>',
        'settle_command': 'sequence<rj_msgs/SettleMotionCommand, 1>',
        'collect_command': 'sequence<rj_msgs/CollectMotionCommand, 1>',
        'line_kick_command': 'sequence<rj_msgs/LineKickMotionCommand, 1>',
        'intercept_command': 'sequence<rj_msgs/InterceptMotionCommand, 1>',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BoundedSequence(rosidl_parser.definition.NamespacedType(['rj_msgs', 'msg'], 'EmptyMotionCommand'), 1),  # noqa: E501
        rosidl_parser.definition.BoundedSequence(rosidl_parser.definition.NamespacedType(['rj_msgs', 'msg'], 'PathTargetMotionCommand'), 1),  # noqa: E501
        rosidl_parser.definition.BoundedSequence(rosidl_parser.definition.NamespacedType(['rj_msgs', 'msg'], 'WorldVelMotionCommand'), 1),  # noqa: E501
        rosidl_parser.definition.BoundedSequence(rosidl_parser.definition.NamespacedType(['rj_msgs', 'msg'], 'PivotMotionCommand'), 1),  # noqa: E501
        rosidl_parser.definition.BoundedSequence(rosidl_parser.definition.NamespacedType(['rj_msgs', 'msg'], 'SettleMotionCommand'), 1),  # noqa: E501
        rosidl_parser.definition.BoundedSequence(rosidl_parser.definition.NamespacedType(['rj_msgs', 'msg'], 'CollectMotionCommand'), 1),  # noqa: E501
        rosidl_parser.definition.BoundedSequence(rosidl_parser.definition.NamespacedType(['rj_msgs', 'msg'], 'LineKickMotionCommand'), 1),  # noqa: E501
        rosidl_parser.definition.BoundedSequence(rosidl_parser.definition.NamespacedType(['rj_msgs', 'msg'], 'InterceptMotionCommand'), 1),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.empty_command = kwargs.get('empty_command', [])
        self.path_target_command = kwargs.get('path_target_command', [])
        self.world_vel_command = kwargs.get('world_vel_command', [])
        self.pivot_command = kwargs.get('pivot_command', [])
        self.settle_command = kwargs.get('settle_command', [])
        self.collect_command = kwargs.get('collect_command', [])
        self.line_kick_command = kwargs.get('line_kick_command', [])
        self.intercept_command = kwargs.get('intercept_command', [])

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
        if self.empty_command != other.empty_command:
            return False
        if self.path_target_command != other.path_target_command:
            return False
        if self.world_vel_command != other.world_vel_command:
            return False
        if self.pivot_command != other.pivot_command:
            return False
        if self.settle_command != other.settle_command:
            return False
        if self.collect_command != other.collect_command:
            return False
        if self.line_kick_command != other.line_kick_command:
            return False
        if self.intercept_command != other.intercept_command:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @property
    def empty_command(self):
        """Message field 'empty_command'."""
        return self._empty_command

    @empty_command.setter
    def empty_command(self, value):
        if __debug__:
            from rj_msgs.msg import EmptyMotionCommand
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
                 all(isinstance(v, EmptyMotionCommand) for v in value) and
                 True), \
                "The 'empty_command' field must be a set or sequence with length <= 1 and each value of type 'EmptyMotionCommand'"
        self._empty_command = value

    @property
    def path_target_command(self):
        """Message field 'path_target_command'."""
        return self._path_target_command

    @path_target_command.setter
    def path_target_command(self, value):
        if __debug__:
            from rj_msgs.msg import PathTargetMotionCommand
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
                 all(isinstance(v, PathTargetMotionCommand) for v in value) and
                 True), \
                "The 'path_target_command' field must be a set or sequence with length <= 1 and each value of type 'PathTargetMotionCommand'"
        self._path_target_command = value

    @property
    def world_vel_command(self):
        """Message field 'world_vel_command'."""
        return self._world_vel_command

    @world_vel_command.setter
    def world_vel_command(self, value):
        if __debug__:
            from rj_msgs.msg import WorldVelMotionCommand
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
                 all(isinstance(v, WorldVelMotionCommand) for v in value) and
                 True), \
                "The 'world_vel_command' field must be a set or sequence with length <= 1 and each value of type 'WorldVelMotionCommand'"
        self._world_vel_command = value

    @property
    def pivot_command(self):
        """Message field 'pivot_command'."""
        return self._pivot_command

    @pivot_command.setter
    def pivot_command(self, value):
        if __debug__:
            from rj_msgs.msg import PivotMotionCommand
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
                 all(isinstance(v, PivotMotionCommand) for v in value) and
                 True), \
                "The 'pivot_command' field must be a set or sequence with length <= 1 and each value of type 'PivotMotionCommand'"
        self._pivot_command = value

    @property
    def settle_command(self):
        """Message field 'settle_command'."""
        return self._settle_command

    @settle_command.setter
    def settle_command(self, value):
        if __debug__:
            from rj_msgs.msg import SettleMotionCommand
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
                 all(isinstance(v, SettleMotionCommand) for v in value) and
                 True), \
                "The 'settle_command' field must be a set or sequence with length <= 1 and each value of type 'SettleMotionCommand'"
        self._settle_command = value

    @property
    def collect_command(self):
        """Message field 'collect_command'."""
        return self._collect_command

    @collect_command.setter
    def collect_command(self, value):
        if __debug__:
            from rj_msgs.msg import CollectMotionCommand
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
                 all(isinstance(v, CollectMotionCommand) for v in value) and
                 True), \
                "The 'collect_command' field must be a set or sequence with length <= 1 and each value of type 'CollectMotionCommand'"
        self._collect_command = value

    @property
    def line_kick_command(self):
        """Message field 'line_kick_command'."""
        return self._line_kick_command

    @line_kick_command.setter
    def line_kick_command(self, value):
        if __debug__:
            from rj_msgs.msg import LineKickMotionCommand
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
                 all(isinstance(v, LineKickMotionCommand) for v in value) and
                 True), \
                "The 'line_kick_command' field must be a set or sequence with length <= 1 and each value of type 'LineKickMotionCommand'"
        self._line_kick_command = value

    @property
    def intercept_command(self):
        """Message field 'intercept_command'."""
        return self._intercept_command

    @intercept_command.setter
    def intercept_command(self, value):
        if __debug__:
            from rj_msgs.msg import InterceptMotionCommand
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
                 all(isinstance(v, InterceptMotionCommand) for v in value) and
                 True), \
                "The 'intercept_command' field must be a set or sequence with length <= 1 and each value of type 'InterceptMotionCommand'"
        self._intercept_command = value
