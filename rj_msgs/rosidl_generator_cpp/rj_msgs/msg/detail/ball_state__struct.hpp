// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from rj_msgs:msg/BallState.idl
// generated code does not contain a copyright notice

#ifndef RJ_MSGS__MSG__DETAIL__BALL_STATE__STRUCT_HPP_
#define RJ_MSGS__MSG__DETAIL__BALL_STATE__STRUCT_HPP_

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
// Member 'position'
// Member 'velocity'
#include "rj_geometry_msgs/msg/detail/point__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__rj_msgs__msg__BallState __attribute__((deprecated))
#else
# define DEPRECATED__rj_msgs__msg__BallState __declspec(deprecated)
#endif

namespace rj_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct BallState_
{
  using Type = BallState_<ContainerAllocator>;

  explicit BallState_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_init),
    position(_init),
    velocity(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->visible = false;
    }
  }

  explicit BallState_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_alloc, _init),
    position(_alloc, _init),
    velocity(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->visible = false;
    }
  }

  // field types and members
  using _stamp_type =
    builtin_interfaces::msg::Time_<ContainerAllocator>;
  _stamp_type stamp;
  using _position_type =
    rj_geometry_msgs::msg::Point_<ContainerAllocator>;
  _position_type position;
  using _velocity_type =
    rj_geometry_msgs::msg::Point_<ContainerAllocator>;
  _velocity_type velocity;
  using _visible_type =
    bool;
  _visible_type visible;

  // setters for named parameter idiom
  Type & set__stamp(
    const builtin_interfaces::msg::Time_<ContainerAllocator> & _arg)
  {
    this->stamp = _arg;
    return *this;
  }
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
  Type & set__visible(
    const bool & _arg)
  {
    this->visible = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    rj_msgs::msg::BallState_<ContainerAllocator> *;
  using ConstRawPtr =
    const rj_msgs::msg::BallState_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rj_msgs::msg::BallState_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rj_msgs::msg::BallState_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rj_msgs::msg::BallState_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rj_msgs::msg::BallState_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rj_msgs::msg::BallState_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rj_msgs::msg::BallState_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rj_msgs::msg::BallState_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rj_msgs::msg::BallState_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rj_msgs__msg__BallState
    std::shared_ptr<rj_msgs::msg::BallState_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rj_msgs__msg__BallState
    std::shared_ptr<rj_msgs::msg::BallState_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const BallState_ & other) const
  {
    if (this->stamp != other.stamp) {
      return false;
    }
    if (this->position != other.position) {
      return false;
    }
    if (this->velocity != other.velocity) {
      return false;
    }
    if (this->visible != other.visible) {
      return false;
    }
    return true;
  }
  bool operator!=(const BallState_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct BallState_

// alias to use template instance with default allocator
using BallState =
  rj_msgs::msg::BallState_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace rj_msgs

#endif  // RJ_MSGS__MSG__DETAIL__BALL_STATE__STRUCT_HPP_
