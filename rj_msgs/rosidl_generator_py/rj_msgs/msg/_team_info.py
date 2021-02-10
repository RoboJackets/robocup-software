# generated from rosidl_generator_py/resource/_idl.py.em
# with input from rj_msgs:msg/TeamInfo.idl
# generated code does not contain a copyright notice


# Import statements for member types

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_TeamInfo(type):
    """Metaclass of message 'TeamInfo'."""

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
                'rj_msgs.msg.TeamInfo')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__team_info
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__team_info
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__team_info
            cls._TYPE_SUPPORT = module.type_support_msg__msg__team_info
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__team_info

            from builtin_interfaces.msg import Duration
            if Duration.__class__._TYPE_SUPPORT is None:
                Duration.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class TeamInfo(metaclass=Metaclass_TeamInfo):
    """Message class 'TeamInfo'."""

    __slots__ = [
        '_name',
        '_score',
        '_num_red_cards',
        '_num_yellow_cards',
        '_yellow_card_remaining_times',
        '_timeouts_left',
        '_remaining_timeout_time',
        '_goalie_id',
    ]

    _fields_and_field_types = {
        'name': 'string',
        'score': 'int32',
        'num_red_cards': 'uint64',
        'num_yellow_cards': 'uint64',
        'yellow_card_remaining_times': 'sequence<builtin_interfaces/Duration>',
        'timeouts_left': 'uint64',
        'remaining_timeout_time': 'builtin_interfaces/Duration',
        'goalie_id': 'uint8',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint64'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint64'),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.NamespacedType(['builtin_interfaces', 'msg'], 'Duration')),  # noqa: E501
        rosidl_parser.definition.BasicType('uint64'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['builtin_interfaces', 'msg'], 'Duration'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.name = kwargs.get('name', str())
        self.score = kwargs.get('score', int())
        self.num_red_cards = kwargs.get('num_red_cards', int())
        self.num_yellow_cards = kwargs.get('num_yellow_cards', int())
        self.yellow_card_remaining_times = kwargs.get('yellow_card_remaining_times', [])
        self.timeouts_left = kwargs.get('timeouts_left', int())
        from builtin_interfaces.msg import Duration
        self.remaining_timeout_time = kwargs.get('remaining_timeout_time', Duration())
        self.goalie_id = kwargs.get('goalie_id', int())

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
        if self.name != other.name:
            return False
        if self.score != other.score:
            return False
        if self.num_red_cards != other.num_red_cards:
            return False
        if self.num_yellow_cards != other.num_yellow_cards:
            return False
        if self.yellow_card_remaining_times != other.yellow_card_remaining_times:
            return False
        if self.timeouts_left != other.timeouts_left:
            return False
        if self.remaining_timeout_time != other.remaining_timeout_time:
            return False
        if self.goalie_id != other.goalie_id:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @property
    def name(self):
        """Message field 'name'."""
        return self._name

    @name.setter
    def name(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'name' field must be of type 'str'"
        self._name = value

    @property
    def score(self):
        """Message field 'score'."""
        return self._score

    @score.setter
    def score(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'score' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'score' field must be an integer in [-2147483648, 2147483647]"
        self._score = value

    @property
    def num_red_cards(self):
        """Message field 'num_red_cards'."""
        return self._num_red_cards

    @num_red_cards.setter
    def num_red_cards(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'num_red_cards' field must be of type 'int'"
            assert value >= 0 and value < 18446744073709551616, \
                "The 'num_red_cards' field must be an unsigned integer in [0, 18446744073709551615]"
        self._num_red_cards = value

    @property
    def num_yellow_cards(self):
        """Message field 'num_yellow_cards'."""
        return self._num_yellow_cards

    @num_yellow_cards.setter
    def num_yellow_cards(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'num_yellow_cards' field must be of type 'int'"
            assert value >= 0 and value < 18446744073709551616, \
                "The 'num_yellow_cards' field must be an unsigned integer in [0, 18446744073709551615]"
        self._num_yellow_cards = value

    @property
    def yellow_card_remaining_times(self):
        """Message field 'yellow_card_remaining_times'."""
        return self._yellow_card_remaining_times

    @yellow_card_remaining_times.setter
    def yellow_card_remaining_times(self, value):
        if __debug__:
            from builtin_interfaces.msg import Duration
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
                 all(isinstance(v, Duration) for v in value) and
                 True), \
                "The 'yellow_card_remaining_times' field must be a set or sequence and each value of type 'Duration'"
        self._yellow_card_remaining_times = value

    @property
    def timeouts_left(self):
        """Message field 'timeouts_left'."""
        return self._timeouts_left

    @timeouts_left.setter
    def timeouts_left(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'timeouts_left' field must be of type 'int'"
            assert value >= 0 and value < 18446744073709551616, \
                "The 'timeouts_left' field must be an unsigned integer in [0, 18446744073709551615]"
        self._timeouts_left = value

    @property
    def remaining_timeout_time(self):
        """Message field 'remaining_timeout_time'."""
        return self._remaining_timeout_time

    @remaining_timeout_time.setter
    def remaining_timeout_time(self, value):
        if __debug__:
            from builtin_interfaces.msg import Duration
            assert \
                isinstance(value, Duration), \
                "The 'remaining_timeout_time' field must be a sub message of type 'Duration'"
        self._remaining_timeout_time = value

    @property
    def goalie_id(self):
        """Message field 'goalie_id'."""
        return self._goalie_id

    @goalie_id.setter
    def goalie_id(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'goalie_id' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'goalie_id' field must be an unsigned integer in [0, 255]"
        self._goalie_id = value
