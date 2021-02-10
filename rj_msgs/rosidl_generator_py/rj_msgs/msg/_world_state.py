# generated from rosidl_generator_py/resource/_idl.py.em
# with input from rj_msgs:msg/WorldState.idl
# generated code does not contain a copyright notice


# Import statements for member types

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_WorldState(type):
    """Metaclass of message 'WorldState'."""

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
                'rj_msgs.msg.WorldState')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__world_state
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__world_state
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__world_state
            cls._TYPE_SUPPORT = module.type_support_msg__msg__world_state
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__world_state

            from builtin_interfaces.msg import Time
            if Time.__class__._TYPE_SUPPORT is None:
                Time.__class__.__import_type_support__()

            from rj_msgs.msg import BallState
            if BallState.__class__._TYPE_SUPPORT is None:
                BallState.__class__.__import_type_support__()

            from rj_msgs.msg import RobotState
            if RobotState.__class__._TYPE_SUPPORT is None:
                RobotState.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class WorldState(metaclass=Metaclass_WorldState):
    """Message class 'WorldState'."""

    __slots__ = [
        '_last_update_time',
        '_their_robots',
        '_our_robots',
        '_ball',
    ]

    _fields_and_field_types = {
        'last_update_time': 'builtin_interfaces/Time',
        'their_robots': 'sequence<rj_msgs/RobotState>',
        'our_robots': 'sequence<rj_msgs/RobotState>',
        'ball': 'rj_msgs/BallState',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['builtin_interfaces', 'msg'], 'Time'),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.NamespacedType(['rj_msgs', 'msg'], 'RobotState')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.NamespacedType(['rj_msgs', 'msg'], 'RobotState')),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['rj_msgs', 'msg'], 'BallState'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from builtin_interfaces.msg import Time
        self.last_update_time = kwargs.get('last_update_time', Time())
        self.their_robots = kwargs.get('their_robots', [])
        self.our_robots = kwargs.get('our_robots', [])
        from rj_msgs.msg import BallState
        self.ball = kwargs.get('ball', BallState())

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
        if self.last_update_time != other.last_update_time:
            return False
        if self.their_robots != other.their_robots:
            return False
        if self.our_robots != other.our_robots:
            return False
        if self.ball != other.ball:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @property
    def last_update_time(self):
        """Message field 'last_update_time'."""
        return self._last_update_time

    @last_update_time.setter
    def last_update_time(self, value):
        if __debug__:
            from builtin_interfaces.msg import Time
            assert \
                isinstance(value, Time), \
                "The 'last_update_time' field must be a sub message of type 'Time'"
        self._last_update_time = value

    @property
    def their_robots(self):
        """Message field 'their_robots'."""
        return self._their_robots

    @their_robots.setter
    def their_robots(self, value):
        if __debug__:
            from rj_msgs.msg import RobotState
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
                 all(isinstance(v, RobotState) for v in value) and
                 True), \
                "The 'their_robots' field must be a set or sequence and each value of type 'RobotState'"
        self._their_robots = value

    @property
    def our_robots(self):
        """Message field 'our_robots'."""
        return self._our_robots

    @our_robots.setter
    def our_robots(self, value):
        if __debug__:
            from rj_msgs.msg import RobotState
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
                 all(isinstance(v, RobotState) for v in value) and
                 True), \
                "The 'our_robots' field must be a set or sequence and each value of type 'RobotState'"
        self._our_robots = value

    @property
    def ball(self):
        """Message field 'ball'."""
        return self._ball

    @ball.setter
    def ball(self, value):
        if __debug__:
            from rj_msgs.msg import BallState
            assert \
                isinstance(value, BallState), \
                "The 'ball' field must be a sub message of type 'BallState'"
        self._ball = value
