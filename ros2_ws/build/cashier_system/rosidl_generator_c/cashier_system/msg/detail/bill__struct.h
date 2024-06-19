// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from cashier_system:msg/Bill.idl
// generated code does not contain a copyright notice

#ifndef CASHIER_SYSTEM__MSG__DETAIL__BILL__STRUCT_H_
#define CASHIER_SYSTEM__MSG__DETAIL__BILL__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'timestamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in msg/Bill in the package cashier_system.
typedef struct cashier_system__msg__Bill
{
  int32_t bill_number;
  builtin_interfaces__msg__Time timestamp;
  int32_t quantity;
  float price;
  float total;
} cashier_system__msg__Bill;

// Struct for a sequence of cashier_system__msg__Bill.
typedef struct cashier_system__msg__Bill__Sequence
{
  cashier_system__msg__Bill * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} cashier_system__msg__Bill__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CASHIER_SYSTEM__MSG__DETAIL__BILL__STRUCT_H_
