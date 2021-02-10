# generated from rosidl_generator_py/resource/_idl.py.em
# with input from rj_msgs:srv/QuickCommands.idl
# generated code does not contain a copyright notice


# Import statements for member types

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_QuickCommands_Request(type):
    """Metaclass of message 'QuickCommands_Request'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'COMMAND_HALT': 0,
        'COMMAND_STOP': 1,
        'COMMAND_READY': 2,
        'COMMAND_PLAY': 3,
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
                'rj_msgs.srv.QuickCommands_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__quick_commands__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__quick_commands__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__quick_commands__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__quick_commands__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__quick_commands__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'COMMAND_HALT': cls.__constants['COMMAND_HALT'],
            'COMMAND_STOP': cls.__constants['COMMAND_STOP'],
            'COMMAND_READY': cls.__constants['COMMAND_READY'],
            'COMMAND_PLAY': cls.__constants['COMMAND_PLAY'],
        }

    @property
    def COMMAND_HALT(self):
        """Message constant 'COMMAND_HALT'."""
        return Metaclass_QuickCommands_Request.__constants['COMMAND_HALT']

    @property
    def COMMAND_STOP(self):
        """Message constant 'COMMAND_STOP'."""
        return Metaclass_QuickCommands_Request.__constants['COMMAND_STOP']

    @property
    def COMMAND_READY(self):
        """Message constant 'COMMAND_READY'."""
        return Metaclass_QuickCommands_Request.__constants['COMMAND_READY']

    @property
    def COMMAND_PLAY(self):
        """Message constant 'COMMAND_PLAY'."""
        return Metaclass_QuickCommands_Request.__constants['COMMAND_PLAY']


class QuickCommands_Request(metaclass=Metaclass_QuickCommands_Request):
    """
    Message class 'QuickCommands_Request'.

    Constants:
      COMMAND_HALT
      COMMAND_STOP
      COMMAND_READY
      COMMAND_PLAY
    """

    __slots__ = [
        '_state',
    ]

    _fields_and_field_types = {
        'state': 'uint8',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.state = kwargs.get('state', int())

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
        if self.state != other.state:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @property
    def state(self):
        """Message field 'state'."""
        return self._state

    @state.setter
    def state(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'state' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'state' field must be an unsigned integer in [0, 255]"
        self._state = value


# Import statements for member types

# already imported above
# import rosidl_parser.definition


class Metaclass_QuickCommands_Response(type):
    """Metaclass of message 'QuickCommands_Response'."""

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
                'rj_msgs.srv.QuickCommands_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__quick_commands__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__quick_commands__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__quick_commands__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__quick_commands__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__quick_commands__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class QuickCommands_Response(metaclass=Metaclass_QuickCommands_Response):
    """Message class 'QuickCommands_Response'."""

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


class Metaclass_QuickCommands(type):
    """Metaclass of service 'QuickCommands'."""

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
                'rj_msgs.srv.QuickCommands')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__quick_commands

            from rj_msgs.srv import _quick_commands
            if _quick_commands.Metaclass_QuickCommands_Request._TYPE_SUPPORT is None:
                _quick_commands.Metaclass_QuickCommands_Request.__import_type_support__()
            if _quick_commands.Metaclass_QuickCommands_Response._TYPE_SUPPORT is None:
                _quick_commands.Metaclass_QuickCommands_Response.__import_type_support__()


class QuickCommands(metaclass=Metaclass_QuickCommands):
    from rj_msgs.srv._quick_commands import QuickCommands_Request as Request
    from rj_msgs.srv._quick_commands import QuickCommands_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
