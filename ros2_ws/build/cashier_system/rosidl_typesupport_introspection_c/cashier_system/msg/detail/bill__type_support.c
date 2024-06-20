// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from cashier_system:msg/Bill.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "cashier_system/msg/detail/bill__rosidl_typesupport_introspection_c.h"
#include "cashier_system/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "cashier_system/msg/detail/bill__functions.h"
#include "cashier_system/msg/detail/bill__struct.h"


// Include directives for member types
// Member `timestamp`
#include "builtin_interfaces/msg/time.h"
// Member `timestamp`
#include "builtin_interfaces/msg/detail/time__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void cashier_system__msg__Bill__rosidl_typesupport_introspection_c__Bill_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  cashier_system__msg__Bill__init(message_memory);
}

void cashier_system__msg__Bill__rosidl_typesupport_introspection_c__Bill_fini_function(void * message_memory)
{
  cashier_system__msg__Bill__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember cashier_system__msg__Bill__rosidl_typesupport_introspection_c__Bill_message_member_array[5] = {
  {
    "bill_number",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(cashier_system__msg__Bill, bill_number),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "timestamp",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(cashier_system__msg__Bill, timestamp),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "quantity",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(cashier_system__msg__Bill, quantity),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "price",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(cashier_system__msg__Bill, price),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "total",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(cashier_system__msg__Bill, total),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers cashier_system__msg__Bill__rosidl_typesupport_introspection_c__Bill_message_members = {
  "cashier_system__msg",  // message namespace
  "Bill",  // message name
  5,  // number of fields
  sizeof(cashier_system__msg__Bill),
  cashier_system__msg__Bill__rosidl_typesupport_introspection_c__Bill_message_member_array,  // message members
  cashier_system__msg__Bill__rosidl_typesupport_introspection_c__Bill_init_function,  // function to initialize message memory (memory has to be allocated)
  cashier_system__msg__Bill__rosidl_typesupport_introspection_c__Bill_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t cashier_system__msg__Bill__rosidl_typesupport_introspection_c__Bill_message_type_support_handle = {
  0,
  &cashier_system__msg__Bill__rosidl_typesupport_introspection_c__Bill_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_cashier_system
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, cashier_system, msg, Bill)() {
  cashier_system__msg__Bill__rosidl_typesupport_introspection_c__Bill_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, builtin_interfaces, msg, Time)();
  if (!cashier_system__msg__Bill__rosidl_typesupport_introspection_c__Bill_message_type_support_handle.typesupport_identifier) {
    cashier_system__msg__Bill__rosidl_typesupport_introspection_c__Bill_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &cashier_system__msg__Bill__rosidl_typesupport_introspection_c__Bill_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
