# generated from rosidl_generator_py/resource/_idl.py.em
# with input from rj_msgs:msg/ManipulatorSetpoint.idl
# generated code does not contain a copyright notice


# Import statements for member types

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_ManipulatorSetpoint(type):
    """Metaclass of message 'ManipulatorSetpoint'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'SHOOT_MODE_KICK': 0,
        'SHOOT_MODE_CHIP': 1,
        'TRIGGER_MODE_STAND_DOWN': 0,
        'TRIGGER_MODE_IMMEDIATE': 1,
        'TRIGGER_MODE_ON_BREAK_BEAM': 2,
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
                'rj_msgs.msg.ManipulatorSetpoint')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__manipulator_setpoint
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__manipulator_setpoint
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__manipulator_setpoint
            cls._TYPE_SUPPORT = module.type_support_msg__msg__manipulator_setpoint
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__manipulator_setpoint

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'SHOOT_MODE_KICK': cls.__constants['SHOOT_MODE_KICK'],
            'SHOOT_MODE_CHIP': cls.__constants['SHOOT_MODE_CHIP'],
            'TRIGGER_MODE_STAND_DOWN': cls.__constants['TRIGGER_MODE_STAND_DOWN'],
            'TRIGGER_MODE_IMMEDIATE': cls.__constants['TRIGGER_MODE_IMMEDIATE'],
            'TRIGGER_MODE_ON_BREAK_BEAM': cls.__constants['TRIGGER_MODE_ON_BREAK_BEAM'],
        }

    @property
    def SHOOT_MODE_KICK(self):
        """Message constant 'SHOOT_MODE_KICK'."""
        return Metaclass_ManipulatorSetpoint.__constants['SHOOT_MODE_KICK']

    @property
    def SHOOT_MODE_CHIP(self):
        """Message constant 'SHOOT_MODE_CHIP'."""
        return Metaclass_ManipulatorSetpoint.__constants['SHOOT_MODE_CHIP']

    @property
    def TRIGGER_MODE_STAND_DOWN(self):
        """Message constant 'TRIGGER_MODE_STAND_DOWN'."""
        return Metaclass_ManipulatorSetpoint.__constants['TRIGGER_MODE_STAND_DOWN']

    @property
    def TRIGGER_MODE_IMMEDIATE(self):
        """Message constant 'TRIGGER_MODE_IMMEDIATE'."""
        return Metaclass_ManipulatorSetpoint.__constants['TRIGGER_MODE_IMMEDIATE']

    @property
    def TRIGGER_MODE_ON_BREAK_BEAM(self):
        """Message constant 'TRIGGER_MODE_ON_BREAK_BEAM'."""
        return Metaclass_ManipulatorSetpoint.__constants['TRIGGER_MODE_ON_BREAK_BEAM']


class ManipulatorSetpoint(metaclass=Metaclass_ManipulatorSetpoint):
    """
    Message class 'ManipulatorSetpoint'.

    Constants:
      SHOOT_MODE_KICK
      SHOOT_MODE_CHIP
      TRIGGER_MODE_STAND_DOWN
      TRIGGER_MODE_IMMEDIATE
      TRIGGER_MODE_ON_BREAK_BEAM
    """

    __slots__ = [
        '_shoot_mode',
        '_trigger_mode',
        '_kick_strength',
        '_dribbler_speed',
    ]

    _fields_and_field_types = {
        'shoot_mode': 'uint8',
        'trigger_mode': 'uint8',
        'kick_strength': 'int8',
        'dribbler_speed': 'float',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('int8'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.shoot_mode = kwargs.get('shoot_mode', int())
        self.trigger_mode = kwargs.get('trigger_mode', int())
        self.kick_strength = kwargs.get('kick_strength', int())
        self.dribbler_speed = kwargs.get('dribbler_speed', float())

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
        if self.shoot_mode != other.shoot_mode:
            return False
        if self.trigger_mode != other.trigger_mode:
            return False
        if self.kick_strength != other.kick_strength:
            return False
        if self.dribbler_speed != other.dribbler_speed:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @property
    def shoot_mode(self):
        """Message field 'shoot_mode'."""
        return self._shoot_mode

    @shoot_mode.setter
    def shoot_mode(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'shoot_mode' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'shoot_mode' field must be an unsigned integer in [0, 255]"
        self._shoot_mode = value

    @property
    def trigger_mode(self):
        """Message field 'trigger_mode'."""
        return self._trigger_mode

    @trigger_mode.setter
    def trigger_mode(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'trigger_mode' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'trigger_mode' field must be an unsigned integer in [0, 255]"
        self._trigger_mode = value

    @property
    def kick_strength(self):
        """Message field 'kick_strength'."""
        return self._kick_strength

    @kick_strength.setter
    def kick_strength(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'kick_strength' field must be of type 'int'"
            assert value >= -128 and value < 128, \
                "The 'kick_strength' field must be an integer in [-128, 127]"
        self._kick_strength = value

    @property
    def dribbler_speed(self):
        """Message field 'dribbler_speed'."""
        return self._dribbler_speed

    @dribbler_speed.setter
    def dribbler_speed(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'dribbler_speed' field must be of type 'float'"
        self._dribbler_speed = value
