// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from cashier_system:msg/Bill.idl
// generated code does not contain a copyright notice

#ifndef CASHIER_SYSTEM__MSG__DETAIL__BILL__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define CASHIER_SYSTEM__MSG__DETAIL__BILL__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "cashier_system/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "cashier_system/msg/detail/bill__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

namespace cashier_system
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_cashier_system
cdr_serialize(
  const cashier_system::msg::Bill & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_cashier_system
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  cashier_system::msg::Bill & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_cashier_system
get_serialized_size(
  const cashier_system::msg::Bill & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_cashier_system
max_serialized_size_Bill(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace cashier_system

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_cashier_system
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, cashier_system, msg, Bill)();

#ifdef __cplusplus
}
#endif

#endif  // CASHIER_SYSTEM__MSG__DETAIL__BILL__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
