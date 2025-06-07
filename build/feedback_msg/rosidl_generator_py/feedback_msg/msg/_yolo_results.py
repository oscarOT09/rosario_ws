# generated from rosidl_generator_py/resource/_idl.py.em
# with input from feedback_msg:msg/YoloResults.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_YoloResults(type):
    """Metaclass of message 'YoloResults'."""

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
            module = import_type_support('feedback_msg')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'feedback_msg.msg.YoloResults')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__yolo_results
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__yolo_results
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__yolo_results
            cls._TYPE_SUPPORT = module.type_support_msg__msg__yolo_results
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__yolo_results

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class YoloResults(metaclass=Metaclass_YoloResults):
    """Message class 'YoloResults'."""

    __slots__ = [
        '_dotted_line',
        '_signal',
        '_traffic_light',
    ]

    _fields_and_field_types = {
        'dotted_line': 'boolean',
        'signal': 'int32',
        'traffic_light': 'int32',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.dotted_line = kwargs.get('dotted_line', bool())
        self.signal = kwargs.get('signal', int())
        self.traffic_light = kwargs.get('traffic_light', int())

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
        if self.dotted_line != other.dotted_line:
            return False
        if self.signal != other.signal:
            return False
        if self.traffic_light != other.traffic_light:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def dotted_line(self):
        """Message field 'dotted_line'."""
        return self._dotted_line

    @dotted_line.setter
    def dotted_line(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'dotted_line' field must be of type 'bool'"
        self._dotted_line = value

    @builtins.property
    def signal(self):
        """Message field 'signal'."""
        return self._signal

    @signal.setter
    def signal(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'signal' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'signal' field must be an integer in [-2147483648, 2147483647]"
        self._signal = value

    @builtins.property
    def traffic_light(self):
        """Message field 'traffic_light'."""
        return self._traffic_light

    @traffic_light.setter
    def traffic_light(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'traffic_light' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'traffic_light' field must be an integer in [-2147483648, 2147483647]"
        self._traffic_light = value
