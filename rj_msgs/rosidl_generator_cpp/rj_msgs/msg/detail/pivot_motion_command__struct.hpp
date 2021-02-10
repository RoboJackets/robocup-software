// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from rj_msgs:msg/PivotMotionCommand.idl
// generated code does not contain a copyright notice

#ifndef RJ_MSGS__MSG__DETAIL__PIVOT_MOTION_COMMAND__STRUCT_HPP_
#define RJ_MSGS__MSG__DETAIL__PIVOT_MOTION_COMMAND__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'pivot_point'
// Member 'pivot_target'
#include "rj_geometry_msgs/msg/detail/point__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__rj_msgs__msg__PivotMotionCommand __attribute__((deprecated))
#else
# define DEPRECATED__rj_msgs__msg__PivotMotionCommand __declspec(deprecated)
#endif

namespace rj_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct PivotMotionCommand_
{
  using Type = PivotMotionCommand_<ContainerAllocator>;

  explicit PivotMotionCommand_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : pivot_point(_init),
    pivot_target(_init)
  {
    (void)_init;
  }

  explicit PivotMotionCommand_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : pivot_point(_alloc, _init),
    pivot_target(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _pivot_point_type =
    rj_geometry_msgs::msg::Point_<ContainerAllocator>;
  _pivot_point_type pivot_point;
  using _pivot_target_type =
    rj_geometry_msgs::msg::Point_<ContainerAllocator>;
  _pivot_target_type pivot_target;

  // setters for named parameter idiom
  Type & set__pivot_point(
    const rj_geometry_msgs::msg::Point_<ContainerAllocator> & _arg)
  {
    this->pivot_point = _arg;
    return *this;
  }
  Type & set__pivot_target(
    const rj_geometry_msgs::msg::Point_<ContainerAllocator> & _arg)
  {
    this->pivot_target = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    rj_msgs::msg::PivotMotionCommand_<ContainerAllocator> *;
  using ConstRawPtr =
    const rj_msgs::msg::PivotMotionCommand_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rj_msgs::msg::PivotMotionCommand_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rj_msgs::msg::PivotMotionCommand_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rj_msgs::msg::PivotMotionCommand_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rj_msgs::msg::PivotMotionCommand_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rj_msgs::msg::PivotMotionCommand_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rj_msgs::msg::PivotMotionCommand_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rj_msgs::msg::PivotMotionCommand_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rj_msgs::msg::PivotMotionCommand_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rj_msgs__msg__PivotMotionCommand
    std::shared_ptr<rj_msgs::msg::PivotMotionCommand_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rj_msgs__msg__PivotMotionCommand
    std::shared_ptr<rj_msgs::msg::PivotMotionCommand_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PivotMotionCommand_ & other) const
  {
    if (this->pivot_point != other.pivot_point) {
      return false;
    }
    if (this->pivot_target != other.pivot_target) {
      return false;
    }
    return true;
  }
  bool operator!=(const PivotMotionCommand_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PivotMotionCommand_

// alias to use template instance with default allocator
using PivotMotionCommand =
  rj_msgs::msg::PivotMotionCommand_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace rj_msgs

#endif  // RJ_MSGS__MSG__DETAIL__PIVOT_MOTION_COMMAND__STRUCT_HPP_
