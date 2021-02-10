// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from rj_msgs:msg/LinearMotionInstant.idl
// generated code does not contain a copyright notice

#ifndef RJ_MSGS__MSG__DETAIL__LINEAR_MOTION_INSTANT__STRUCT_HPP_
#define RJ_MSGS__MSG__DETAIL__LINEAR_MOTION_INSTANT__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'position'
// Member 'velocity'
#include "rj_geometry_msgs/msg/detail/point__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__rj_msgs__msg__LinearMotionInstant __attribute__((deprecated))
#else
# define DEPRECATED__rj_msgs__msg__LinearMotionInstant __declspec(deprecated)
#endif

namespace rj_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct LinearMotionInstant_
{
  using Type = LinearMotionInstant_<ContainerAllocator>;

  explicit LinearMotionInstant_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : position(_init),
    velocity(_init)
  {
    (void)_init;
  }

  explicit LinearMotionInstant_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : position(_alloc, _init),
    velocity(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _position_type =
    rj_geometry_msgs::msg::Point_<ContainerAllocator>;
  _position_type position;
  using _velocity_type =
    rj_geometry_msgs::msg::Point_<ContainerAllocator>;
  _velocity_type velocity;

  // setters for named parameter idiom
  Type & set__position(
    const rj_geometry_msgs::msg::Point_<ContainerAllocator> & _arg)
  {
    this->position = _arg;
    return *this;
  }
  Type & set__velocity(
    const rj_geometry_msgs::msg::Point_<ContainerAllocator> & _arg)
  {
    this->velocity = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    rj_msgs::msg::LinearMotionInstant_<ContainerAllocator> *;
  using ConstRawPtr =
    const rj_msgs::msg::LinearMotionInstant_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rj_msgs::msg::LinearMotionInstant_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rj_msgs::msg::LinearMotionInstant_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rj_msgs::msg::LinearMotionInstant_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rj_msgs::msg::LinearMotionInstant_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rj_msgs::msg::LinearMotionInstant_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rj_msgs::msg::LinearMotionInstant_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rj_msgs::msg::LinearMotionInstant_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rj_msgs::msg::LinearMotionInstant_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rj_msgs__msg__LinearMotionInstant
    std::shared_ptr<rj_msgs::msg::LinearMotionInstant_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rj_msgs__msg__LinearMotionInstant
    std::shared_ptr<rj_msgs::msg::LinearMotionInstant_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const LinearMotionInstant_ & other) const
  {
    if (this->position != other.position) {
      return false;
    }
    if (this->velocity != other.velocity) {
      return false;
    }
    return true;
  }
  bool operator!=(const LinearMotionInstant_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct LinearMotionInstant_

// alias to use template instance with default allocator
using LinearMotionInstant =
  rj_msgs::msg::LinearMotionInstant_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace rj_msgs

#endif  // RJ_MSGS__MSG__DETAIL__LINEAR_MOTION_INSTANT__STRUCT_HPP_
