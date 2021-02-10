# generated from rosidl_generator_py/resource/_idl.py.em
# with input from rj_msgs:msg/DetectionRobot.idl
# generated code does not contain a copyright notice


# Import statements for member types

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_DetectionRobot(type):
    """Metaclass of message 'DetectionRobot'."""

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
                'rj_msgs.msg.DetectionRobot')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__detection_robot
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__detection_robot
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__detection_robot
            cls._TYPE_SUPPORT = module.type_support_msg__msg__detection_robot
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__detection_robot

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class DetectionRobot(metaclass=Metaclass_DetectionRobot):
    """Message class 'DetectionRobot'."""

    __slots__ = [
        '_confidence',
        '_robot_id',
        '_x',
        '_y',
        '_orientation',
        '_pixel_x',
        '_pixel_y',
        '_height',
    ]

    _fields_and_field_types = {
        'confidence': 'float',
        'robot_id': 'uint32',
        'x': 'float',
        'y': 'float',
        'orientation': 'float',
        'pixel_x': 'float',
        'pixel_y': 'float',
        'height': 'float',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.confidence = kwargs.get('confidence', float())
        self.robot_id = kwargs.get('robot_id', int())
        self.x = kwargs.get('x', float())
        self.y = kwargs.get('y', float())
        self.orientation = kwargs.get('orientation', float())
        self.pixel_x = kwargs.get('pixel_x', float())
        self.pixel_y = kwargs.get('pixel_y', float())
        self.height = kwargs.get('height', float())

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
        if self.confidence != other.confidence:
            return False
        if self.robot_id != other.robot_id:
            return False
        if self.x != other.x:
            return False
        if self.y != other.y:
            return False
        if self.orientation != other.orientation:
            return False
        if self.pixel_x != other.pixel_x:
            return False
        if self.pixel_y != other.pixel_y:
            return False
        if self.height != other.height:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @property
    def confidence(self):
        """Message field 'confidence'."""
        return self._confidence

    @confidence.setter
    def confidence(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'confidence' field must be of type 'float'"
        self._confidence = value

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
            assert value >= 0 and value < 4294967296, \
                "The 'robot_id' field must be an unsigned integer in [0, 4294967295]"
        self._robot_id = value

    @property
    def x(self):
        """Message field 'x'."""
        return self._x

    @x.setter
    def x(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'x' field must be of type 'float'"
        self._x = value

    @property
    def y(self):
        """Message field 'y'."""
        return self._y

    @y.setter
    def y(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'y' field must be of type 'float'"
        self._y = value

    @property
    def orientation(self):
        """Message field 'orientation'."""
        return self._orientation

    @orientation.setter
    def orientation(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'orientation' field must be of type 'float'"
        self._orientation = value

    @property
    def pixel_x(self):
        """Message field 'pixel_x'."""
        return self._pixel_x

    @pixel_x.setter
    def pixel_x(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'pixel_x' field must be of type 'float'"
        self._pixel_x = value

    @property
    def pixel_y(self):
        """Message field 'pixel_y'."""
        return self._pixel_y

    @pixel_y.setter
    def pixel_y(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'pixel_y' field must be of type 'float'"
        self._pixel_y = value

    @property
    def height(self):
        """Message field 'height'."""
        return self._height

    @height.setter
    def height(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'height' field must be of type 'float'"
        self._height = value
