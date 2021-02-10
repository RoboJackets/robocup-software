# generated from rosidl_generator_py/resource/_idl.py.em
# with input from rj_msgs:msg/FieldDimensions.idl
# generated code does not contain a copyright notice


# Import statements for member types

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_FieldDimensions(type):
    """Metaclass of message 'FieldDimensions'."""

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
                'rj_msgs.msg.FieldDimensions')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__field_dimensions
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__field_dimensions
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__field_dimensions
            cls._TYPE_SUPPORT = module.type_support_msg__msg__field_dimensions
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__field_dimensions

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class FieldDimensions(metaclass=Metaclass_FieldDimensions):
    """Message class 'FieldDimensions'."""

    __slots__ = [
        '_length',
        '_width',
        '_border',
        '_line_width',
        '_goal_width',
        '_goal_depth',
        '_goal_height',
        '_penalty_short_dist',
        '_penalty_long_dist',
        '_center_radius',
        '_center_diameter',
        '_goal_flat',
        '_floor_length',
        '_floor_width',
    ]

    _fields_and_field_types = {
        'length': 'float',
        'width': 'float',
        'border': 'float',
        'line_width': 'float',
        'goal_width': 'float',
        'goal_depth': 'float',
        'goal_height': 'float',
        'penalty_short_dist': 'float',
        'penalty_long_dist': 'float',
        'center_radius': 'float',
        'center_diameter': 'float',
        'goal_flat': 'float',
        'floor_length': 'float',
        'floor_width': 'float',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
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
        self.length = kwargs.get('length', float())
        self.width = kwargs.get('width', float())
        self.border = kwargs.get('border', float())
        self.line_width = kwargs.get('line_width', float())
        self.goal_width = kwargs.get('goal_width', float())
        self.goal_depth = kwargs.get('goal_depth', float())
        self.goal_height = kwargs.get('goal_height', float())
        self.penalty_short_dist = kwargs.get('penalty_short_dist', float())
        self.penalty_long_dist = kwargs.get('penalty_long_dist', float())
        self.center_radius = kwargs.get('center_radius', float())
        self.center_diameter = kwargs.get('center_diameter', float())
        self.goal_flat = kwargs.get('goal_flat', float())
        self.floor_length = kwargs.get('floor_length', float())
        self.floor_width = kwargs.get('floor_width', float())

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
        if self.length != other.length:
            return False
        if self.width != other.width:
            return False
        if self.border != other.border:
            return False
        if self.line_width != other.line_width:
            return False
        if self.goal_width != other.goal_width:
            return False
        if self.goal_depth != other.goal_depth:
            return False
        if self.goal_height != other.goal_height:
            return False
        if self.penalty_short_dist != other.penalty_short_dist:
            return False
        if self.penalty_long_dist != other.penalty_long_dist:
            return False
        if self.center_radius != other.center_radius:
            return False
        if self.center_diameter != other.center_diameter:
            return False
        if self.goal_flat != other.goal_flat:
            return False
        if self.floor_length != other.floor_length:
            return False
        if self.floor_width != other.floor_width:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @property
    def length(self):
        """Message field 'length'."""
        return self._length

    @length.setter
    def length(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'length' field must be of type 'float'"
        self._length = value

    @property
    def width(self):
        """Message field 'width'."""
        return self._width

    @width.setter
    def width(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'width' field must be of type 'float'"
        self._width = value

    @property
    def border(self):
        """Message field 'border'."""
        return self._border

    @border.setter
    def border(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'border' field must be of type 'float'"
        self._border = value

    @property
    def line_width(self):
        """Message field 'line_width'."""
        return self._line_width

    @line_width.setter
    def line_width(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'line_width' field must be of type 'float'"
        self._line_width = value

    @property
    def goal_width(self):
        """Message field 'goal_width'."""
        return self._goal_width

    @goal_width.setter
    def goal_width(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'goal_width' field must be of type 'float'"
        self._goal_width = value

    @property
    def goal_depth(self):
        """Message field 'goal_depth'."""
        return self._goal_depth

    @goal_depth.setter
    def goal_depth(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'goal_depth' field must be of type 'float'"
        self._goal_depth = value

    @property
    def goal_height(self):
        """Message field 'goal_height'."""
        return self._goal_height

    @goal_height.setter
    def goal_height(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'goal_height' field must be of type 'float'"
        self._goal_height = value

    @property
    def penalty_short_dist(self):
        """Message field 'penalty_short_dist'."""
        return self._penalty_short_dist

    @penalty_short_dist.setter
    def penalty_short_dist(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'penalty_short_dist' field must be of type 'float'"
        self._penalty_short_dist = value

    @property
    def penalty_long_dist(self):
        """Message field 'penalty_long_dist'."""
        return self._penalty_long_dist

    @penalty_long_dist.setter
    def penalty_long_dist(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'penalty_long_dist' field must be of type 'float'"
        self._penalty_long_dist = value

    @property
    def center_radius(self):
        """Message field 'center_radius'."""
        return self._center_radius

    @center_radius.setter
    def center_radius(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'center_radius' field must be of type 'float'"
        self._center_radius = value

    @property
    def center_diameter(self):
        """Message field 'center_diameter'."""
        return self._center_diameter

    @center_diameter.setter
    def center_diameter(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'center_diameter' field must be of type 'float'"
        self._center_diameter = value

    @property
    def goal_flat(self):
        """Message field 'goal_flat'."""
        return self._goal_flat

    @goal_flat.setter
    def goal_flat(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'goal_flat' field must be of type 'float'"
        self._goal_flat = value

    @property
    def floor_length(self):
        """Message field 'floor_length'."""
        return self._floor_length

    @floor_length.setter
    def floor_length(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'floor_length' field must be of type 'float'"
        self._floor_length = value

    @property
    def floor_width(self):
        """Message field 'floor_width'."""
        return self._floor_width

    @floor_width.setter
    def floor_width(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'floor_width' field must be of type 'float'"
        self._floor_width = value
