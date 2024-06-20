// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from cashier_system:msg/Bill.idl
// generated code does not contain a copyright notice

#ifndef CASHIER_SYSTEM__MSG__DETAIL__BILL__STRUCT_HPP_
#define CASHIER_SYSTEM__MSG__DETAIL__BILL__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'timestamp'
#include "builtin_interfaces/msg/detail/time__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__cashier_system__msg__Bill __attribute__((deprecated))
#else
# define DEPRECATED__cashier_system__msg__Bill __declspec(deprecated)
#endif

namespace cashier_system
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Bill_
{
  using Type = Bill_<ContainerAllocator>;

  explicit Bill_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : timestamp(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->bill_number = 0l;
      this->quantity = 0l;
      this->price = 0.0f;
      this->total = 0.0f;
    }
  }

  explicit Bill_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : timestamp(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->bill_number = 0l;
      this->quantity = 0l;
      this->price = 0.0f;
      this->total = 0.0f;
    }
  }

  // field types and members
  using _bill_number_type =
    int32_t;
  _bill_number_type bill_number;
  using _timestamp_type =
    builtin_interfaces::msg::Time_<ContainerAllocator>;
  _timestamp_type timestamp;
  using _quantity_type =
    int32_t;
  _quantity_type quantity;
  using _price_type =
    float;
  _price_type price;
  using _total_type =
    float;
  _total_type total;

  // setters for named parameter idiom
  Type & set__bill_number(
    const int32_t & _arg)
  {
    this->bill_number = _arg;
    return *this;
  }
  Type & set__timestamp(
    const builtin_interfaces::msg::Time_<ContainerAllocator> & _arg)
  {
    this->timestamp = _arg;
    return *this;
  }
  Type & set__quantity(
    const int32_t & _arg)
  {
    this->quantity = _arg;
    return *this;
  }
  Type & set__price(
    const float & _arg)
  {
    this->price = _arg;
    return *this;
  }
  Type & set__total(
    const float & _arg)
  {
    this->total = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    cashier_system::msg::Bill_<ContainerAllocator> *;
  using ConstRawPtr =
    const cashier_system::msg::Bill_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<cashier_system::msg::Bill_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<cashier_system::msg::Bill_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      cashier_system::msg::Bill_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<cashier_system::msg::Bill_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      cashier_system::msg::Bill_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<cashier_system::msg::Bill_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<cashier_system::msg::Bill_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<cashier_system::msg::Bill_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__cashier_system__msg__Bill
    std::shared_ptr<cashier_system::msg::Bill_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__cashier_system__msg__Bill
    std::shared_ptr<cashier_system::msg::Bill_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Bill_ & other) const
  {
    if (this->bill_number != other.bill_number) {
      return false;
    }
    if (this->timestamp != other.timestamp) {
      return false;
    }
    if (this->quantity != other.quantity) {
      return false;
    }
    if (this->price != other.price) {
      return false;
    }
    if (this->total != other.total) {
      return false;
    }
    return true;
  }
  bool operator!=(const Bill_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Bill_

// alias to use template instance with default allocator
using Bill =
  cashier_system::msg::Bill_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace cashier_system

#endif  // CASHIER_SYSTEM__MSG__DETAIL__BILL__STRUCT_HPP_
