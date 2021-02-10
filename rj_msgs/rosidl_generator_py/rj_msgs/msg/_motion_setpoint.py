# generated from rosidl_generator_py/resource/_idl.py.em
# with input from rj_msgs:msg/MotionSetpoint.idl
# generated code does not contain a copyright notice


# Import statements for member types

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_MotionSetpoint(type):
    """Metaclass of message 'MotionSetpoint'."""

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
                'rj_msgs.msg.MotionSetpoint')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__motion_setpoint
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__motion_setpoint
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__motion_setpoint
            cls._TYPE_SUPPORT = module.type_support_msg__msg__motion_setpoint
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__motion_setpoint

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class MotionSetpoint(metaclass=Metaclass_MotionSetpoint):
    """Message class 'MotionSetpoint'."""

    __slots__ = [
        '_velocity_x_mps',
        '_velocity_y_mps',
        '_velocity_z_radps',
    ]

    _fields_and_field_types = {
        'velocity_x_mps': 'double',
        'velocity_y_mps': 'double',
        'velocity_z_radps': 'double',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.velocity_x_mps = kwargs.get('velocity_x_mps', float())
        self.velocity_y_mps = kwargs.get('velocity_y_mps', float())
        self.velocity_z_radps = kwargs.get('velocity_z_radps', float())

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
        if self.velocity_x_mps != other.velocity_x_mps:
            return False
        if self.velocity_y_mps != other.velocity_y_mps:
            return False
        if self.velocity_z_radps != other.velocity_z_radps:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @property
    def velocity_x_mps(self):
        """Message field 'velocity_x_mps'."""
        return self._velocity_x_mps

    @velocity_x_mps.setter
    def velocity_x_mps(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'velocity_x_mps' field must be of type 'float'"
        self._velocity_x_mps = value

    @property
    def velocity_y_mps(self):
        """Message field 'velocity_y_mps'."""
        return self._velocity_y_mps

    @velocity_y_mps.setter
    def velocity_y_mps(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'velocity_y_mps' field must be of type 'float'"
        self._velocity_y_mps = value

    @property
    def velocity_z_radps(self):
        """Message field 'velocity_z_radps'."""
        return self._velocity_z_radps

    @velocity_z_radps.setter
    def velocity_z_radps(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'velocity_z_radps' field must be of type 'float'"
        self._velocity_z_radps = value
