# generated from rosidl_generator_py/resource/_idl.py.em
# with input from rj_msgs:msg/GameSettings.idl
# generated code does not contain a copyright notice


# Import statements for member types

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_GameSettings(type):
    """Metaclass of message 'GameSettings'."""

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
                'rj_msgs.msg.GameSettings')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__game_settings
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__game_settings
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__game_settings
            cls._TYPE_SUPPORT = module.type_support_msg__msg__game_settings
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__game_settings

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class GameSettings(metaclass=Metaclass_GameSettings):
    """Message class 'GameSettings'."""

    __slots__ = [
        '_simulation',
        '_request_blue_team',
        '_request_goalie_id',
        '_defend_plus_x',
        '_use_our_half',
        '_use_their_half',
        '_paused',
    ]

    _fields_and_field_types = {
        'simulation': 'boolean',
        'request_blue_team': 'boolean',
        'request_goalie_id': 'int32',
        'defend_plus_x': 'boolean',
        'use_our_half': 'boolean',
        'use_their_half': 'boolean',
        'paused': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.simulation = kwargs.get('simulation', bool())
        self.request_blue_team = kwargs.get('request_blue_team', bool())
        self.request_goalie_id = kwargs.get('request_goalie_id', int())
        self.defend_plus_x = kwargs.get('defend_plus_x', bool())
        self.use_our_half = kwargs.get('use_our_half', bool())
        self.use_their_half = kwargs.get('use_their_half', bool())
        self.paused = kwargs.get('paused', bool())

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
        if self.simulation != other.simulation:
            return False
        if self.request_blue_team != other.request_blue_team:
            return False
        if self.request_goalie_id != other.request_goalie_id:
            return False
        if self.defend_plus_x != other.defend_plus_x:
            return False
        if self.use_our_half != other.use_our_half:
            return False
        if self.use_their_half != other.use_their_half:
            return False
        if self.paused != other.paused:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @property
    def simulation(self):
        """Message field 'simulation'."""
        return self._simulation

    @simulation.setter
    def simulation(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'simulation' field must be of type 'bool'"
        self._simulation = value

    @property
    def request_blue_team(self):
        """Message field 'request_blue_team'."""
        return self._request_blue_team

    @request_blue_team.setter
    def request_blue_team(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'request_blue_team' field must be of type 'bool'"
        self._request_blue_team = value

    @property
    def request_goalie_id(self):
        """Message field 'request_goalie_id'."""
        return self._request_goalie_id

    @request_goalie_id.setter
    def request_goalie_id(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'request_goalie_id' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'request_goalie_id' field must be an integer in [-2147483648, 2147483647]"
        self._request_goalie_id = value

    @property
    def defend_plus_x(self):
        """Message field 'defend_plus_x'."""
        return self._defend_plus_x

    @defend_plus_x.setter
    def defend_plus_x(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'defend_plus_x' field must be of type 'bool'"
        self._defend_plus_x = value

    @property
    def use_our_half(self):
        """Message field 'use_our_half'."""
        return self._use_our_half

    @use_our_half.setter
    def use_our_half(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'use_our_half' field must be of type 'bool'"
        self._use_our_half = value

    @property
    def use_their_half(self):
        """Message field 'use_their_half'."""
        return self._use_their_half

    @use_their_half.setter
    def use_their_half(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'use_their_half' field must be of type 'bool'"
        self._use_their_half = value

    @property
    def paused(self):
        """Message field 'paused'."""
        return self._paused

    @paused.setter
    def paused(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'paused' field must be of type 'bool'"
        self._paused = value
