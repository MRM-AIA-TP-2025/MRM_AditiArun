// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from cashier_system:msg/Bill.idl
// generated code does not contain a copyright notice

#ifndef CASHIER_SYSTEM__MSG__DETAIL__BILL__BUILDER_HPP_
#define CASHIER_SYSTEM__MSG__DETAIL__BILL__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "cashier_system/msg/detail/bill__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace cashier_system
{

namespace msg
{

namespace builder
{

class Init_Bill_total
{
public:
  explicit Init_Bill_total(::cashier_system::msg::Bill & msg)
  : msg_(msg)
  {}
  ::cashier_system::msg::Bill total(::cashier_system::msg::Bill::_total_type arg)
  {
    msg_.total = std::move(arg);
    return std::move(msg_);
  }

private:
  ::cashier_system::msg::Bill msg_;
};

class Init_Bill_price
{
public:
  explicit Init_Bill_price(::cashier_system::msg::Bill & msg)
  : msg_(msg)
  {}
  Init_Bill_total price(::cashier_system::msg::Bill::_price_type arg)
  {
    msg_.price = std::move(arg);
    return Init_Bill_total(msg_);
  }

private:
  ::cashier_system::msg::Bill msg_;
};

class Init_Bill_quantity
{
public:
  explicit Init_Bill_quantity(::cashier_system::msg::Bill & msg)
  : msg_(msg)
  {}
  Init_Bill_price quantity(::cashier_system::msg::Bill::_quantity_type arg)
  {
    msg_.quantity = std::move(arg);
    return Init_Bill_price(msg_);
  }

private:
  ::cashier_system::msg::Bill msg_;
};

class Init_Bill_timestamp
{
public:
  explicit Init_Bill_timestamp(::cashier_system::msg::Bill & msg)
  : msg_(msg)
  {}
  Init_Bill_quantity timestamp(::cashier_system::msg::Bill::_timestamp_type arg)
  {
    msg_.timestamp = std::move(arg);
    return Init_Bill_quantity(msg_);
  }

private:
  ::cashier_system::msg::Bill msg_;
};

class Init_Bill_bill_number
{
public:
  Init_Bill_bill_number()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Bill_timestamp bill_number(::cashier_system::msg::Bill::_bill_number_type arg)
  {
    msg_.bill_number = std::move(arg);
    return Init_Bill_timestamp(msg_);
  }

private:
  ::cashier_system::msg::Bill msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::cashier_system::msg::Bill>()
{
  return cashier_system::msg::builder::Init_Bill_bill_number();
}

}  // namespace cashier_system

#endif  // CASHIER_SYSTEM__MSG__DETAIL__BILL__BUILDER_HPP_
