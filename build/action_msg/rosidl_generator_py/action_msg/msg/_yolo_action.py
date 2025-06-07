# generated from rosidl_generator_py/resource/_idl.py.em
# with input from action_msg:msg/YoloAction.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_YoloAction(type):
    """Metaclass of message 'YoloAction'."""

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
            module = import_type_support('action_msg')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'action_msg.msg.YoloAction')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__yolo_action
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__yolo_action
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__yolo_action
            cls._TYPE_SUPPORT = module.type_support_msg__msg__yolo_action
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__yolo_action

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class YoloAction(metaclass=Metaclass_YoloAction):
    """Message class 'YoloAction'."""

    __slots__ = [
        '_rojo',
        '_amarillo',
        '_verde',
        '_adelante',
        '_girar_l',
        '_girar_r',
        '_trabajo',
        '_ceder',
        '_alto',
    ]

    _fields_and_field_types = {
        'rojo': 'boolean',
        'amarillo': 'boolean',
        'verde': 'boolean',
        'adelante': 'boolean',
        'girar_l': 'boolean',
        'girar_r': 'boolean',
        'trabajo': 'boolean',
        'ceder': 'boolean',
        'alto': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.rojo = kwargs.get('rojo', bool())
        self.amarillo = kwargs.get('amarillo', bool())
        self.verde = kwargs.get('verde', bool())
        self.adelante = kwargs.get('adelante', bool())
        self.girar_l = kwargs.get('girar_l', bool())
        self.girar_r = kwargs.get('girar_r', bool())
        self.trabajo = kwargs.get('trabajo', bool())
        self.ceder = kwargs.get('ceder', bool())
        self.alto = kwargs.get('alto', bool())

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
        if self.rojo != other.rojo:
            return False
        if self.amarillo != other.amarillo:
            return False
        if self.verde != other.verde:
            return False
        if self.adelante != other.adelante:
            return False
        if self.girar_l != other.girar_l:
            return False
        if self.girar_r != other.girar_r:
            return False
        if self.trabajo != other.trabajo:
            return False
        if self.ceder != other.ceder:
            return False
        if self.alto != other.alto:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def rojo(self):
        """Message field 'rojo'."""
        return self._rojo

    @rojo.setter
    def rojo(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'rojo' field must be of type 'bool'"
        self._rojo = value

    @builtins.property
    def amarillo(self):
        """Message field 'amarillo'."""
        return self._amarillo

    @amarillo.setter
    def amarillo(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'amarillo' field must be of type 'bool'"
        self._amarillo = value

    @builtins.property
    def verde(self):
        """Message field 'verde'."""
        return self._verde

    @verde.setter
    def verde(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'verde' field must be of type 'bool'"
        self._verde = value

    @builtins.property
    def adelante(self):
        """Message field 'adelante'."""
        return self._adelante

    @adelante.setter
    def adelante(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'adelante' field must be of type 'bool'"
        self._adelante = value

    @builtins.property
    def girar_l(self):
        """Message field 'girar_l'."""
        return self._girar_l

    @girar_l.setter
    def girar_l(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'girar_l' field must be of type 'bool'"
        self._girar_l = value

    @builtins.property
    def girar_r(self):
        """Message field 'girar_r'."""
        return self._girar_r

    @girar_r.setter
    def girar_r(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'girar_r' field must be of type 'bool'"
        self._girar_r = value

    @builtins.property
    def trabajo(self):
        """Message field 'trabajo'."""
        return self._trabajo

    @trabajo.setter
    def trabajo(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'trabajo' field must be of type 'bool'"
        self._trabajo = value

    @builtins.property
    def ceder(self):
        """Message field 'ceder'."""
        return self._ceder

    @ceder.setter
    def ceder(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'ceder' field must be of type 'bool'"
        self._ceder = value

    @builtins.property
    def alto(self):
        """Message field 'alto'."""
        return self._alto

    @alto.setter
    def alto(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'alto' field must be of type 'bool'"
        self._alto = value
