# generated from rosidl_generator_py/resource/_idl.py.em
# with input from rj_msgs:srv/QuickRestart.idl
# generated code does not contain a copyright notice


# Import statements for member types

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_QuickRestart_Request(type):
    """Metaclass of message 'QuickRestart_Request'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'RESTART_KICKOFF': 0,
        'RESTART_DIRECT': 1,
        'RESTART_INDIRECT': 2,
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
                'rj_msgs.srv.QuickRestart_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__quick_restart__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__quick_restart__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__quick_restart__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__quick_restart__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__quick_restart__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'RESTART_KICKOFF': cls.__constants['RESTART_KICKOFF'],
            'RESTART_DIRECT': cls.__constants['RESTART_DIRECT'],
            'RESTART_INDIRECT': cls.__constants['RESTART_INDIRECT'],
        }

    @property
    def RESTART_KICKOFF(self):
        """Message constant 'RESTART_KICKOFF'."""
        return Metaclass_QuickRestart_Request.__constants['RESTART_KICKOFF']

    @property
    def RESTART_DIRECT(self):
        """Message constant 'RESTART_DIRECT'."""
        return Metaclass_QuickRestart_Request.__constants['RESTART_DIRECT']

    @property
    def RESTART_INDIRECT(self):
        """Message constant 'RESTART_INDIRECT'."""
        return Metaclass_QuickRestart_Request.__constants['RESTART_INDIRECT']


class QuickRestart_Request(metaclass=Metaclass_QuickRestart_Request):
    """
    Message class 'QuickRestart_Request'.

    Constants:
      RESTART_KICKOFF
      RESTART_DIRECT
      RESTART_INDIRECT
    """

    __slots__ = [
        '_restart',
        '_blue_team',
    ]

    _fields_and_field_types = {
        'restart': 'uint8',
        'blue_team': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.restart = kwargs.get('restart', int())
        self.blue_team = kwargs.get('blue_team', bool())

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
        if self.restart != other.restart:
            return False
        if self.blue_team != other.blue_team:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

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
    def blue_team(self):
        """Message field 'blue_team'."""
        return self._blue_team

    @blue_team.setter
    def blue_team(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'blue_team' field must be of type 'bool'"
        self._blue_team = value


# Import statements for member types

# already imported above
# import rosidl_parser.definition


class Metaclass_QuickRestart_Response(type):
    """Metaclass of message 'QuickRestart_Response'."""

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
                'rj_msgs.srv.QuickRestart_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__quick_restart__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__quick_restart__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__quick_restart__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__quick_restart__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__quick_restart__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class QuickRestart_Response(metaclass=Metaclass_QuickRestart_Response):
    """Message class 'QuickRestart_Response'."""

    __slots__ = [
    ]

    _fields_and_field_types = {
    }

    SLOT_TYPES = (
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))

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
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)


class Metaclass_QuickRestart(type):
    """Metaclass of service 'QuickRestart'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('rj_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'rj_msgs.srv.QuickRestart')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__quick_restart

            from rj_msgs.srv import _quick_restart
            if _quick_restart.Metaclass_QuickRestart_Request._TYPE_SUPPORT is None:
                _quick_restart.Metaclass_QuickRestart_Request.__import_type_support__()
            if _quick_restart.Metaclass_QuickRestart_Response._TYPE_SUPPORT is None:
                _quick_restart.Metaclass_QuickRestart_Response.__import_type_support__()


class QuickRestart(metaclass=Metaclass_QuickRestart):
    from rj_msgs.srv._quick_restart import QuickRestart_Request as Request
    from rj_msgs.srv._quick_restart import QuickRestart_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
