# generated from rosidl_generator_py/resource/_idl.py.em
# with input from rj_msgs:msg/RobotIntent.idl
# generated code does not contain a copyright notice


# Import statements for member types

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_RobotIntent(type):
    """Metaclass of message 'RobotIntent'."""

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
                'rj_msgs.msg.RobotIntent')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__robot_intent
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__robot_intent
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__robot_intent
            cls._TYPE_SUPPORT = module.type_support_msg__msg__robot_intent
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__robot_intent

            from rj_geometry_msgs.msg import ShapeSet
            if ShapeSet.__class__._TYPE_SUPPORT is None:
                ShapeSet.__class__.__import_type_support__()

            from rj_msgs.msg import MotionCommand
            if MotionCommand.__class__._TYPE_SUPPORT is None:
                MotionCommand.__class__.__import_type_support__()

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
        return Metaclass_RobotIntent.__constants['SHOOT_MODE_KICK']

    @property
    def SHOOT_MODE_CHIP(self):
        """Message constant 'SHOOT_MODE_CHIP'."""
        return Metaclass_RobotIntent.__constants['SHOOT_MODE_CHIP']

    @property
    def TRIGGER_MODE_STAND_DOWN(self):
        """Message constant 'TRIGGER_MODE_STAND_DOWN'."""
        return Metaclass_RobotIntent.__constants['TRIGGER_MODE_STAND_DOWN']

    @property
    def TRIGGER_MODE_IMMEDIATE(self):
        """Message constant 'TRIGGER_MODE_IMMEDIATE'."""
        return Metaclass_RobotIntent.__constants['TRIGGER_MODE_IMMEDIATE']

    @property
    def TRIGGER_MODE_ON_BREAK_BEAM(self):
        """Message constant 'TRIGGER_MODE_ON_BREAK_BEAM'."""
        return Metaclass_RobotIntent.__constants['TRIGGER_MODE_ON_BREAK_BEAM']


class RobotIntent(metaclass=Metaclass_RobotIntent):
    """
    Message class 'RobotIntent'.

    Constants:
      SHOOT_MODE_KICK
      SHOOT_MODE_CHIP
      TRIGGER_MODE_STAND_DOWN
      TRIGGER_MODE_IMMEDIATE
      TRIGGER_MODE_ON_BREAK_BEAM
    """

    __slots__ = [
        '_motion_command',
        '_local_obstacles',
        '_shoot_mode',
        '_trigger_mode',
        '_kick_speed',
        '_dribbler_speed',
        '_is_active',
        '_priority',
    ]

    _fields_and_field_types = {
        'motion_command': 'rj_msgs/MotionCommand',
        'local_obstacles': 'rj_geometry_msgs/ShapeSet',
        'shoot_mode': 'uint8',
        'trigger_mode': 'uint8',
        'kick_speed': 'float',
        'dribbler_speed': 'float',
        'is_active': 'boolean',
        'priority': 'int8',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['rj_msgs', 'msg'], 'MotionCommand'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['rj_geometry_msgs', 'msg'], 'ShapeSet'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('int8'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from rj_msgs.msg import MotionCommand
        self.motion_command = kwargs.get('motion_command', MotionCommand())
        from rj_geometry_msgs.msg import ShapeSet
        self.local_obstacles = kwargs.get('local_obstacles', ShapeSet())
        self.shoot_mode = kwargs.get('shoot_mode', int())
        self.trigger_mode = kwargs.get('trigger_mode', int())
        self.kick_speed = kwargs.get('kick_speed', float())
        self.dribbler_speed = kwargs.get('dribbler_speed', float())
        self.is_active = kwargs.get('is_active', bool())
        self.priority = kwargs.get('priority', int())

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
        if self.motion_command != other.motion_command:
            return False
        if self.local_obstacles != other.local_obstacles:
            return False
        if self.shoot_mode != other.shoot_mode:
            return False
        if self.trigger_mode != other.trigger_mode:
            return False
        if self.kick_speed != other.kick_speed:
            return False
        if self.dribbler_speed != other.dribbler_speed:
            return False
        if self.is_active != other.is_active:
            return False
        if self.priority != other.priority:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @property
    def motion_command(self):
        """Message field 'motion_command'."""
        return self._motion_command

    @motion_command.setter
    def motion_command(self, value):
        if __debug__:
            from rj_msgs.msg import MotionCommand
            assert \
                isinstance(value, MotionCommand), \
                "The 'motion_command' field must be a sub message of type 'MotionCommand'"
        self._motion_command = value

    @property
    def local_obstacles(self):
        """Message field 'local_obstacles'."""
        return self._local_obstacles

    @local_obstacles.setter
    def local_obstacles(self, value):
        if __debug__:
            from rj_geometry_msgs.msg import ShapeSet
            assert \
                isinstance(value, ShapeSet), \
                "The 'local_obstacles' field must be a sub message of type 'ShapeSet'"
        self._local_obstacles = value

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
    def kick_speed(self):
        """Message field 'kick_speed'."""
        return self._kick_speed

    @kick_speed.setter
    def kick_speed(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'kick_speed' field must be of type 'float'"
        self._kick_speed = value

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

    @property
    def is_active(self):
        """Message field 'is_active'."""
        return self._is_active

    @is_active.setter
    def is_active(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'is_active' field must be of type 'bool'"
        self._is_active = value

    @property
    def priority(self):
        """Message field 'priority'."""
        return self._priority

    @priority.setter
    def priority(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'priority' field must be of type 'int'"
            assert value >= -128 and value < 128, \
                "The 'priority' field must be an integer in [-128, 127]"
        self._priority = value
