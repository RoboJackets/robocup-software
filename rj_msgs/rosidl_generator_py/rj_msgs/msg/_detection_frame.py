# generated from rosidl_generator_py/resource/_idl.py.em
# with input from rj_msgs:msg/DetectionFrame.idl
# generated code does not contain a copyright notice


# Import statements for member types

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_DetectionFrame(type):
    """Metaclass of message 'DetectionFrame'."""

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
                'rj_msgs.msg.DetectionFrame')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__detection_frame
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__detection_frame
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__detection_frame
            cls._TYPE_SUPPORT = module.type_support_msg__msg__detection_frame
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__detection_frame

            from builtin_interfaces.msg import Time
            if Time.__class__._TYPE_SUPPORT is None:
                Time.__class__.__import_type_support__()

            from rj_msgs.msg import DetectionBall
            if DetectionBall.__class__._TYPE_SUPPORT is None:
                DetectionBall.__class__.__import_type_support__()

            from rj_msgs.msg import DetectionRobot
            if DetectionRobot.__class__._TYPE_SUPPORT is None:
                DetectionRobot.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class DetectionFrame(metaclass=Metaclass_DetectionFrame):
    """Message class 'DetectionFrame'."""

    __slots__ = [
        '_frame_number',
        '_t_capture',
        '_t_sent',
        '_t_received',
        '_camera_id',
        '_balls',
        '_robots_yellow',
        '_robots_blue',
    ]

    _fields_and_field_types = {
        'frame_number': 'uint32',
        't_capture': 'builtin_interfaces/Time',
        't_sent': 'builtin_interfaces/Time',
        't_received': 'builtin_interfaces/Time',
        'camera_id': 'uint32',
        'balls': 'sequence<rj_msgs/DetectionBall>',
        'robots_yellow': 'sequence<rj_msgs/DetectionRobot>',
        'robots_blue': 'sequence<rj_msgs/DetectionRobot>',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['builtin_interfaces', 'msg'], 'Time'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['builtin_interfaces', 'msg'], 'Time'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['builtin_interfaces', 'msg'], 'Time'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.NamespacedType(['rj_msgs', 'msg'], 'DetectionBall')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.NamespacedType(['rj_msgs', 'msg'], 'DetectionRobot')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.NamespacedType(['rj_msgs', 'msg'], 'DetectionRobot')),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.frame_number = kwargs.get('frame_number', int())
        from builtin_interfaces.msg import Time
        self.t_capture = kwargs.get('t_capture', Time())
        from builtin_interfaces.msg import Time
        self.t_sent = kwargs.get('t_sent', Time())
        from builtin_interfaces.msg import Time
        self.t_received = kwargs.get('t_received', Time())
        self.camera_id = kwargs.get('camera_id', int())
        self.balls = kwargs.get('balls', [])
        self.robots_yellow = kwargs.get('robots_yellow', [])
        self.robots_blue = kwargs.get('robots_blue', [])

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
        if self.frame_number != other.frame_number:
            return False
        if self.t_capture != other.t_capture:
            return False
        if self.t_sent != other.t_sent:
            return False
        if self.t_received != other.t_received:
            return False
        if self.camera_id != other.camera_id:
            return False
        if self.balls != other.balls:
            return False
        if self.robots_yellow != other.robots_yellow:
            return False
        if self.robots_blue != other.robots_blue:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @property
    def frame_number(self):
        """Message field 'frame_number'."""
        return self._frame_number

    @frame_number.setter
    def frame_number(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'frame_number' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'frame_number' field must be an unsigned integer in [0, 4294967295]"
        self._frame_number = value

    @property
    def t_capture(self):
        """Message field 't_capture'."""
        return self._t_capture

    @t_capture.setter
    def t_capture(self, value):
        if __debug__:
            from builtin_interfaces.msg import Time
            assert \
                isinstance(value, Time), \
                "The 't_capture' field must be a sub message of type 'Time'"
        self._t_capture = value

    @property
    def t_sent(self):
        """Message field 't_sent'."""
        return self._t_sent

    @t_sent.setter
    def t_sent(self, value):
        if __debug__:
            from builtin_interfaces.msg import Time
            assert \
                isinstance(value, Time), \
                "The 't_sent' field must be a sub message of type 'Time'"
        self._t_sent = value

    @property
    def t_received(self):
        """Message field 't_received'."""
        return self._t_received

    @t_received.setter
    def t_received(self, value):
        if __debug__:
            from builtin_interfaces.msg import Time
            assert \
                isinstance(value, Time), \
                "The 't_received' field must be a sub message of type 'Time'"
        self._t_received = value

    @property
    def camera_id(self):
        """Message field 'camera_id'."""
        return self._camera_id

    @camera_id.setter
    def camera_id(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'camera_id' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'camera_id' field must be an unsigned integer in [0, 4294967295]"
        self._camera_id = value

    @property
    def balls(self):
        """Message field 'balls'."""
        return self._balls

    @balls.setter
    def balls(self, value):
        if __debug__:
            from rj_msgs.msg import DetectionBall
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
                 all(isinstance(v, DetectionBall) for v in value) and
                 True), \
                "The 'balls' field must be a set or sequence and each value of type 'DetectionBall'"
        self._balls = value

    @property
    def robots_yellow(self):
        """Message field 'robots_yellow'."""
        return self._robots_yellow

    @robots_yellow.setter
    def robots_yellow(self, value):
        if __debug__:
            from rj_msgs.msg import DetectionRobot
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
                 all(isinstance(v, DetectionRobot) for v in value) and
                 True), \
                "The 'robots_yellow' field must be a set or sequence and each value of type 'DetectionRobot'"
        self._robots_yellow = value

    @property
    def robots_blue(self):
        """Message field 'robots_blue'."""
        return self._robots_blue

    @robots_blue.setter
    def robots_blue(self, value):
        if __debug__:
            from rj_msgs.msg import DetectionRobot
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
                 all(isinstance(v, DetectionRobot) for v in value) and
                 True), \
                "The 'robots_blue' field must be a set or sequence and each value of type 'DetectionRobot'"
        self._robots_blue = value
