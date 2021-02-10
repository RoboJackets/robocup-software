// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from rj_msgs:msg/RobotInstant.idl
// generated code does not contain a copyright notice

#ifndef RJ_MSGS__MSG__DETAIL__ROBOT_INSTANT__STRUCT_HPP_
#define RJ_MSGS__MSG__DETAIL__ROBOT_INSTANT__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.hpp"
// Member 'pose'
#include "rj_geometry_msgs/msg/detail/pose__struct.hpp"
// Member 'velocity'
#include "rj_geometry_msgs/msg/detail/twist__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__rj_msgs__msg__RobotInstant __attribute__((deprecated))
#else
# define DEPRECATED__rj_msgs__msg__RobotInstant __declspec(deprecated)
#endif

namespace rj_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct RobotInstant_
{
  using Type = RobotInstant_<ContainerAllocator>;

  explicit RobotInstant_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_init),
    pose(_init),
    velocity(_init)
  {
    (void)_init;
  }

  explicit RobotInstant_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_alloc, _init),
    pose(_alloc, _init),
    velocity(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _stamp_type =
    builtin_interfaces::msg::Time_<ContainerAllocator>;
  _stamp_type stamp;
  using _pose_type =
    rj_geometry_msgs::msg::Pose_<ContainerAllocator>;
  _pose_type pose;
  using _velocity_type =
    rj_geometry_msgs::msg::Twist_<ContainerAllocator>;
  _velocity_type velocity;

  // setters for named parameter idiom
  Type & set__stamp(
    const builtin_interfaces::msg::Time_<ContainerAllocator> & _arg)
  {
    this->stamp = _arg;
    return *this;
  }
  Type & set__pose(
    const rj_geometry_msgs::msg::Pose_<ContainerAllocator> & _arg)
  {
    this->pose = _arg;
    return *this;
  }
  Type & set__velocity(
    const rj_geometry_msgs::msg::Twist_<ContainerAllocator> & _arg)
  {
    this->velocity = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    rj_msgs::msg::RobotInstant_<ContainerAllocator> *;
  using ConstRawPtr =
    const rj_msgs::msg::RobotInstant_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rj_msgs::msg::RobotInstant_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rj_msgs::msg::RobotInstant_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rj_msgs::msg::RobotInstant_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rj_msgs::msg::RobotInstant_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rj_msgs::msg::RobotInstant_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rj_msgs::msg::RobotInstant_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rj_msgs::msg::RobotInstant_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rj_msgs::msg::RobotInstant_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rj_msgs__msg__RobotInstant
    std::shared_ptr<rj_msgs::msg::RobotInstant_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rj_msgs__msg__RobotInstant
    std::shared_ptr<rj_msgs::msg::RobotInstant_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RobotInstant_ & other) const
  {
    if (this->stamp != other.stamp) {
      return false;
    }
    if (this->pose != other.pose) {
      return false;
    }
    if (this->velocity != other.velocity) {
      return false;
    }
    return true;
  }
  bool operator!=(const RobotInstant_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RobotInstant_

// alias to use template instance with default allocator
using RobotInstant =
  rj_msgs::msg::RobotInstant_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace rj_msgs

#endif  // RJ_MSGS__MSG__DETAIL__ROBOT_INSTANT__STRUCT_HPP_
