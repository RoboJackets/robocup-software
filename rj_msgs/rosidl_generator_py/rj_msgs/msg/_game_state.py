# generated from rosidl_generator_py/resource/_idl.py.em
# with input from rj_msgs:msg/GameState.idl
# generated code does not contain a copyright notice


# Import statements for member types

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_GameState(type):
    """Metaclass of message 'GameState'."""

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
                'rj_msgs.msg.GameState')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__game_state
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__game_state
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__game_state
            cls._TYPE_SUPPORT = module.type_support_msg__msg__game_state
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__game_state

            from builtin_interfaces.msg import Duration
            if Duration.__class__._TYPE_SUPPORT is None:
                Duration.__class__.__import_type_support__()

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


class GameState(metaclass=Metaclass_GameState):
    """Message class 'GameState'."""

    __slots__ = [
        '_period',
        '_state',
        '_restart',
        '_our_restart',
        '_stage_time_left',
        '_placement_point',
    ]

    _fields_and_field_types = {
        'period': 'uint8',
        'state': 'uint8',
        'restart': 'uint8',
        'our_restart': 'boolean',
        'stage_time_left': 'builtin_interfaces/Duration',
        'placement_point': 'rj_geometry_msgs/Point',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['builtin_interfaces', 'msg'], 'Duration'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['rj_geometry_msgs', 'msg'], 'Point'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.period = kwargs.get('period', int())
        self.state = kwargs.get('state', int())
        self.restart = kwargs.get('restart', int())
        self.our_restart = kwargs.get('our_restart', bool())
        from builtin_interfaces.msg import Duration
        self.stage_time_left = kwargs.get('stage_time_left', Duration())
        from rj_geometry_msgs.msg import Point
        self.placement_point = kwargs.get('placement_point', Point())

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
        if self.period != other.period:
            return False
        if self.state != other.state:
            return False
        if self.restart != other.restart:
            return False
        if self.our_restart != other.our_restart:
            return False
        if self.stage_time_left != other.stage_time_left:
            return False
        if self.placement_point != other.placement_point:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @property
    def period(self):
        """Message field 'period'."""
        return self._period

    @period.setter
    def period(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'period' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'period' field must be an unsigned integer in [0, 255]"
        self._period = value

    @property
    def state(self):
        """Message field 'state'."""
        return self._state

    @state.setter
    def state(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'state' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'state' field must be an unsigned integer in [0, 255]"
        self._state = value

    @property
    def restart(self):
        """Message field 'restart'."""
        return self._restart

    @restart.setter
    def restart(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'restart' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'restart' field must be an unsigned integer in [0, 255]"
        self._restart = value

    @property
    def our_restart(self):
        """Message field 'our_restart'."""
        return self._our_restart

    @our_restart.setter
    def our_restart(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'our_restart' field must be of type 'bool'"
        self._our_restart = value

    @property
    def stage_time_left(self):
        """Message field 'stage_time_left'."""
        return self._stage_time_left

    @stage_time_left.setter
    def stage_time_left(self, value):
        if __debug__:
            from builtin_interfaces.msg import Duration
            assert \
                isinstance(value, Duration), \
                "The 'stage_time_left' field must be a sub message of type 'Duration'"
        self._stage_time_left = value

    @property
    def placement_point(self):
        """Message field 'placement_point'."""
        return self._placement_point

    @placement_point.setter
    def placement_point(self, value):
        if __debug__:
            from rj_geometry_msgs.msg import Point
            assert \
                isinstance(value, Point), \
                "The 'placement_point' field must be a sub message of type 'Point'"
        self._placement_point = value
