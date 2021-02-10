// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from rj_msgs:msg/FieldOrientation.idl
// generated code does not contain a copyright notice

#ifndef RJ_MSGS__MSG__DETAIL__FIELD_ORIENTATION__STRUCT_HPP_
#define RJ_MSGS__MSG__DETAIL__FIELD_ORIENTATION__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__rj_msgs__msg__FieldOrientation __attribute__((deprecated))
#else
# define DEPRECATED__rj_msgs__msg__FieldOrientation __declspec(deprecated)
#endif

namespace rj_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct FieldOrientation_
{
  using Type = FieldOrientation_<ContainerAllocator>;

  explicit FieldOrientation_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->defend_plus_x = false;
    }
  }

  explicit FieldOrientation_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->defend_plus_x = false;
    }
  }

  // field types and members
  using _defend_plus_x_type =
    bool;
  _defend_plus_x_type defend_plus_x;

  // setters for named parameter idiom
  Type & set__defend_plus_x(
    const bool & _arg)
  {
    this->defend_plus_x = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    rj_msgs::msg::FieldOrientation_<ContainerAllocator> *;
  using ConstRawPtr =
    const rj_msgs::msg::FieldOrientation_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rj_msgs::msg::FieldOrientation_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rj_msgs::msg::FieldOrientation_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rj_msgs::msg::FieldOrientation_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rj_msgs::msg::FieldOrientation_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rj_msgs::msg::FieldOrientation_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rj_msgs::msg::FieldOrientation_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rj_msgs::msg::FieldOrientation_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rj_msgs::msg::FieldOrientation_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rj_msgs__msg__FieldOrientation
    std::shared_ptr<rj_msgs::msg::FieldOrientation_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rj_msgs__msg__FieldOrientation
    std::shared_ptr<rj_msgs::msg::FieldOrientation_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const FieldOrientation_ & other) const
  {
    if (this->defend_plus_x != other.defend_plus_x) {
      return false;
    }
    return true;
  }
  bool operator!=(const FieldOrientation_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct FieldOrientation_

// alias to use template instance with default allocator
using FieldOrientation =
  rj_msgs::msg::FieldOrientation_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace rj_msgs

#endif  // RJ_MSGS__MSG__DETAIL__FIELD_ORIENTATION__STRUCT_HPP_
