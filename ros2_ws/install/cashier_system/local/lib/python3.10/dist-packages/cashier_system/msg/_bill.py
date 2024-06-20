# generated from rosidl_generator_py/resource/_idl.py.em
# with input from cashier_system:msg/Bill.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_Bill(type):
    """Metaclass of message 'Bill'."""

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
            module = import_type_support('cashier_system')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'cashier_system.msg.Bill')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__bill
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__bill
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__bill
            cls._TYPE_SUPPORT = module.type_support_msg__msg__bill
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__bill

            from builtin_interfaces.msg import Time
            if Time.__class__._TYPE_SUPPORT is None:
                Time.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class Bill(metaclass=Metaclass_Bill):
    """Message class 'Bill'."""

    __slots__ = [
        '_bill_number',
        '_timestamp',
        '_quantity',
        '_price',
        '_total',
    ]

    _fields_and_field_types = {
        'bill_number': 'int32',
        'timestamp': 'builtin_interfaces/Time',
        'quantity': 'int32',
        'price': 'float',
        'total': 'float',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['builtin_interfaces', 'msg'], 'Time'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.bill_number = kwargs.get('bill_number', int())
        from builtin_interfaces.msg import Time
        self.timestamp = kwargs.get('timestamp', Time())
        self.quantity = kwargs.get('quantity', int())
        self.price = kwargs.get('price', float())
        self.total = kwargs.get('total', float())

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
        if self.bill_number != other.bill_number:
            return False
        if self.timestamp != other.timestamp:
            return False
        if self.quantity != other.quantity:
            return False
        if self.price != other.price:
            return False
        if self.total != other.total:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def bill_number(self):
        """Message field 'bill_number'."""
        return self._bill_number

    @bill_number.setter
    def bill_number(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'bill_number' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'bill_number' field must be an integer in [-2147483648, 2147483647]"
        self._bill_number = value

    @builtins.property
    def timestamp(self):
        """Message field 'timestamp'."""
        return self._timestamp

    @timestamp.setter
    def timestamp(self, value):
        if __debug__:
            from builtin_interfaces.msg import Time
            assert \
                isinstance(value, Time), \
                "The 'timestamp' field must be a sub message of type 'Time'"
        self._timestamp = value

    @builtins.property
    def quantity(self):
        """Message field 'quantity'."""
        return self._quantity

    @quantity.setter
    def quantity(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'quantity' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'quantity' field must be an integer in [-2147483648, 2147483647]"
        self._quantity = value

    @builtins.property
    def price(self):
        """Message field 'price'."""
        return self._price

    @price.setter
    def price(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'price' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'price' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._price = value

    @builtins.property
    def total(self):
        """Message field 'total'."""
        return self._total

    @total.setter
    def total(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'total' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'total' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._total = value
