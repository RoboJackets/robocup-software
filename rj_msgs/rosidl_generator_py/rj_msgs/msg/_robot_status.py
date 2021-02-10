# generated from rosidl_generator_py/resource/_idl.py.em
# with input from rj_msgs:msg/RobotStatus.idl
# generated code does not contain a copyright notice


# Import statements for member types

# Member 'encoder_deltas'
import numpy  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_RobotStatus(type):
    """Metaclass of message 'RobotStatus'."""

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
                'rj_msgs.msg.RobotStatus')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__robot_status
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__robot_status
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__robot_status
            cls._TYPE_SUPPORT = module.type_support_msg__msg__robot_status
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__robot_status

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class RobotStatus(metaclass=Metaclass_RobotStatus):
    """Message class 'RobotStatus'."""

    __slots__ = [
        '_robot_id',
        '_battery_voltage',
        '_motor_errors',
        '_has_ball_sense',
        '_kicker_charged',
        '_kicker_healthy',
        '_fpga_error',
        '_encoder_deltas',
    ]

    _fields_and_field_types = {
        'robot_id': 'uint8',
        'battery_voltage': 'double',
        'motor_errors': 'boolean[5]',
        'has_ball_sense': 'boolean',
        'kicker_charged': 'boolean',
        'kicker_healthy': 'boolean',
        'fpga_error': 'boolean',
        'encoder_deltas': 'int16[4]',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('boolean'), 5),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('int16'), 4),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.robot_id = kwargs.get('robot_id', int())
        self.battery_voltage = kwargs.get('battery_voltage', float())
        self.motor_errors = kwargs.get(
            'motor_errors',
            [bool() for x in range(5)]
        )
        self.has_ball_sense = kwargs.get('has_ball_sense', bool())
        self.kicker_charged = kwargs.get('kicker_charged', bool())
        self.kicker_healthy = kwargs.get('kicker_healthy', bool())
        self.fpga_error = kwargs.get('fpga_error', bool())
        if 'encoder_deltas' not in kwargs:
            self.encoder_deltas = numpy.zeros(4, dtype=numpy.int16)
        else:
            self.encoder_deltas = numpy.array(kwargs.get('encoder_deltas'), dtype=numpy.int16)
            assert self.encoder_deltas.shape == (4, )

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
        if self.robot_id != other.robot_id:
            return False
        if self.battery_voltage != other.battery_voltage:
            return False
        if self.motor_errors != other.motor_errors:
            return False
        if self.has_ball_sense != other.has_ball_sense:
            return False
        if self.kicker_charged != other.kicker_charged:
            return False
        if self.kicker_healthy != other.kicker_healthy:
            return False
        if self.fpga_error != other.fpga_error:
            return False
        if all(self.encoder_deltas != other.encoder_deltas):
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @property
    def robot_id(self):
        """Message field 'robot_id'."""
        return self._robot_id

    @robot_id.setter
    def robot_id(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'robot_id' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'robot_id' field must be an unsigned integer in [0, 255]"
        self._robot_id = value

    @property
    def battery_voltage(self):
        """Message field 'battery_voltage'."""
        return self._battery_voltage

    @battery_voltage.setter
    def battery_voltage(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'battery_voltage' field must be of type 'float'"
        self._battery_voltage = value

    @property
    def motor_errors(self):
        """Message field 'motor_errors'."""
        return self._motor_errors

    @motor_errors.setter
    def motor_errors(self, value):
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
                 len(value) == 5 and
                 all(isinstance(v, bool) for v in value) and
                 True), \
                "The 'motor_errors' field must be a set or sequence with length 5 and each value of type 'bool'"
        self._motor_errors = value

    @property
    def has_ball_sense(self):
        """Message field 'has_ball_sense'."""
        return self._has_ball_sense

    @has_ball_sense.setter
    def has_ball_sense(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'has_ball_sense' field must be of type 'bool'"
        self._has_ball_sense = value

    @property
    def kicker_charged(self):
        """Message field 'kicker_charged'."""
        return self._kicker_charged

    @kicker_charged.setter
    def kicker_charged(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'kicker_charged' field must be of type 'bool'"
        self._kicker_charged = value

    @property
    def kicker_healthy(self):
        """Message field 'kicker_healthy'."""
        return self._kicker_healthy

    @kicker_healthy.setter
    def kicker_healthy(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'kicker_healthy' field must be of type 'bool'"
        self._kicker_healthy = value

    @property
    def fpga_error(self):
        """Message field 'fpga_error'."""
        return self._fpga_error

    @fpga_error.setter
    def fpga_error(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'fpga_error' field must be of type 'bool'"
        self._fpga_error = value

    @property
    def encoder_deltas(self):
        """Message field 'encoder_deltas'."""
        return self._encoder_deltas

    @encoder_deltas.setter
    def encoder_deltas(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.int16, \
                "The 'encoder_deltas' numpy.ndarray() must have the dtype of 'numpy.int16'"
            assert value.size == 4, \
                "The 'encoder_deltas' numpy.ndarray() must have a size of 4"
            self._encoder_deltas = value
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
                 len(value) == 4 and
                 all(isinstance(v, int) for v in value) and
                 all(val >= -32768 and val < 32768 for val in value)), \
                "The 'encoder_deltas' field must be a set or sequence with length 4 and each value of type 'int' and each integer in [-32768, 32767]"
        self._encoder_deltas = numpy.array(value, dtype=numpy.int16)
