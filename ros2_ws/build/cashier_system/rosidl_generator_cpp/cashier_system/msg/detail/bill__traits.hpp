// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from cashier_system:msg/Bill.idl
// generated code does not contain a copyright notice

#ifndef CASHIER_SYSTEM__MSG__DETAIL__BILL__TRAITS_HPP_
#define CASHIER_SYSTEM__MSG__DETAIL__BILL__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "cashier_system/msg/detail/bill__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'timestamp'
#include "builtin_interfaces/msg/detail/time__traits.hpp"

namespace cashier_system
{

namespace msg
{

inline void to_flow_style_yaml(
  const Bill & msg,
  std::ostream & out)
{
  out << "{";
  // member: bill_number
  {
    out << "bill_number: ";
    rosidl_generator_traits::value_to_yaml(msg.bill_number, out);
    out << ", ";
  }

  // member: timestamp
  {
    out << "timestamp: ";
    to_flow_style_yaml(msg.timestamp, out);
    out << ", ";
  }

  // member: quantity
  {
    out << "quantity: ";
    rosidl_generator_traits::value_to_yaml(msg.quantity, out);
    out << ", ";
  }

  // member: price
  {
    out << "price: ";
    rosidl_generator_traits::value_to_yaml(msg.price, out);
    out << ", ";
  }

  // member: total
  {
    out << "total: ";
    rosidl_generator_traits::value_to_yaml(msg.total, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Bill & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: bill_number
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "bill_number: ";
    rosidl_generator_traits::value_to_yaml(msg.bill_number, out);
    out << "\n";
  }

  // member: timestamp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "timestamp:\n";
    to_block_style_yaml(msg.timestamp, out, indentation + 2);
  }

  // member: quantity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "quantity: ";
    rosidl_generator_traits::value_to_yaml(msg.quantity, out);
    out << "\n";
  }

  // member: price
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "price: ";
    rosidl_generator_traits::value_to_yaml(msg.price, out);
    out << "\n";
  }

  // member: total
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "total: ";
    rosidl_generator_traits::value_to_yaml(msg.total, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Bill & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace cashier_system

namespace rosidl_generator_traits
{

[[deprecated("use cashier_system::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const cashier_system::msg::Bill & msg,
  std::ostream & out, size_t indentation = 0)
{
  cashier_system::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use cashier_system::msg::to_yaml() instead")]]
inline std::string to_yaml(const cashier_system::msg::Bill & msg)
{
  return cashier_system::msg::to_yaml(msg);
}

template<>
inline const char * data_type<cashier_system::msg::Bill>()
{
  return "cashier_system::msg::Bill";
}

template<>
inline const char * name<cashier_system::msg::Bill>()
{
  return "cashier_system/msg/Bill";
}

template<>
struct has_fixed_size<cashier_system::msg::Bill>
  : std::integral_constant<bool, has_fixed_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct has_bounded_size<cashier_system::msg::Bill>
  : std::integral_constant<bool, has_bounded_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct is_message<cashier_system::msg::Bill>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // CASHIER_SYSTEM__MSG__DETAIL__BILL__TRAITS_HPP_
