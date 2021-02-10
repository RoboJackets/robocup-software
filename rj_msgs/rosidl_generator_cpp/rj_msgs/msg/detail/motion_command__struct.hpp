// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from rj_msgs:msg/MotionCommand.idl
// generated code does not contain a copyright notice

#ifndef RJ_MSGS__MSG__DETAIL__MOTION_COMMAND__STRUCT_HPP_
#define RJ_MSGS__MSG__DETAIL__MOTION_COMMAND__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'empty_command'
#include "rj_msgs/msg/detail/empty_motion_command__struct.hpp"
// Member 'path_target_command'
#include "rj_msgs/msg/detail/path_target_motion_command__struct.hpp"
// Member 'world_vel_command'
#include "rj_msgs/msg/detail/world_vel_motion_command__struct.hpp"
// Member 'pivot_command'
#include "rj_msgs/msg/detail/pivot_motion_command__struct.hpp"
// Member 'settle_command'
#include "rj_msgs/msg/detail/settle_motion_command__struct.hpp"
// Member 'collect_command'
#include "rj_msgs/msg/detail/collect_motion_command__struct.hpp"
// Member 'line_kick_command'
#include "rj_msgs/msg/detail/line_kick_motion_command__struct.hpp"
// Member 'intercept_command'
#include "rj_msgs/msg/detail/intercept_motion_command__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__rj_msgs__msg__MotionCommand __attribute__((deprecated))
#else
# define DEPRECATED__rj_msgs__msg__MotionCommand __declspec(deprecated)
#endif

namespace rj_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MotionCommand_
{
  using Type = MotionCommand_<ContainerAllocator>;

  explicit MotionCommand_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit MotionCommand_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _empty_command_type =
    rosidl_runtime_cpp::BoundedVector<rj_msgs::msg::EmptyMotionCommand_<ContainerAllocator>, 1, typename ContainerAllocator::template rebind<rj_msgs::msg::EmptyMotionCommand_<ContainerAllocator>>::other>;
  _empty_command_type empty_command;
  using _path_target_command_type =
    rosidl_runtime_cpp::BoundedVector<rj_msgs::msg::PathTargetMotionCommand_<ContainerAllocator>, 1, typename ContainerAllocator::template rebind<rj_msgs::msg::PathTargetMotionCommand_<ContainerAllocator>>::other>;
  _path_target_command_type path_target_command;
  using _world_vel_command_type =
    rosidl_runtime_cpp::BoundedVector<rj_msgs::msg::WorldVelMotionCommand_<ContainerAllocator>, 1, typename ContainerAllocator::template rebind<rj_msgs::msg::WorldVelMotionCommand_<ContainerAllocator>>::other>;
  _world_vel_command_type world_vel_command;
  using _pivot_command_type =
    rosidl_runtime_cpp::BoundedVector<rj_msgs::msg::PivotMotionCommand_<ContainerAllocator>, 1, typename ContainerAllocator::template rebind<rj_msgs::msg::PivotMotionCommand_<ContainerAllocator>>::other>;
  _pivot_command_type pivot_command;
  using _settle_command_type =
    rosidl_runtime_cpp::BoundedVector<rj_msgs::msg::SettleMotionCommand_<ContainerAllocator>, 1, typename ContainerAllocator::template rebind<rj_msgs::msg::SettleMotionCommand_<ContainerAllocator>>::other>;
  _settle_command_type settle_command;
  using _collect_command_type =
    rosidl_runtime_cpp::BoundedVector<rj_msgs::msg::CollectMotionCommand_<ContainerAllocator>, 1, typename ContainerAllocator::template rebind<rj_msgs::msg::CollectMotionCommand_<ContainerAllocator>>::other>;
  _collect_command_type collect_command;
  using _line_kick_command_type =
    rosidl_runtime_cpp::BoundedVector<rj_msgs::msg::LineKickMotionCommand_<ContainerAllocator>, 1, typename ContainerAllocator::template rebind<rj_msgs::msg::LineKickMotionCommand_<ContainerAllocator>>::other>;
  _line_kick_command_type line_kick_command;
  using _intercept_command_type =
    rosidl_runtime_cpp::BoundedVector<rj_msgs::msg::InterceptMotionCommand_<ContainerAllocator>, 1, typename ContainerAllocator::template rebind<rj_msgs::msg::InterceptMotionCommand_<ContainerAllocator>>::other>;
  _intercept_command_type intercept_command;

  // setters for named parameter idiom
  Type & set__empty_command(
    const rosidl_runtime_cpp::BoundedVector<rj_msgs::msg::EmptyMotionCommand_<ContainerAllocator>, 1, typename ContainerAllocator::template rebind<rj_msgs::msg::EmptyMotionCommand_<ContainerAllocator>>::other> & _arg)
  {
    this->empty_command = _arg;
    return *this;
  }
  Type & set__path_target_command(
    const rosidl_runtime_cpp::BoundedVector<rj_msgs::msg::PathTargetMotionCommand_<ContainerAllocator>, 1, typename ContainerAllocator::template rebind<rj_msgs::msg::PathTargetMotionCommand_<ContainerAllocator>>::other> & _arg)
  {
    this->path_target_command = _arg;
    return *this;
  }
  Type & set__world_vel_command(
    const rosidl_runtime_cpp::BoundedVector<rj_msgs::msg::WorldVelMotionCommand_<ContainerAllocator>, 1, typename ContainerAllocator::template rebind<rj_msgs::msg::WorldVelMotionCommand_<ContainerAllocator>>::other> & _arg)
  {
    this->world_vel_command = _arg;
    return *this;
  }
  Type & set__pivot_command(
    const rosidl_runtime_cpp::BoundedVector<rj_msgs::msg::PivotMotionCommand_<ContainerAllocator>, 1, typename ContainerAllocator::template rebind<rj_msgs::msg::PivotMotionCommand_<ContainerAllocator>>::other> & _arg)
  {
    this->pivot_command = _arg;
    return *this;
  }
  Type & set__settle_command(
    const rosidl_runtime_cpp::BoundedVector<rj_msgs::msg::SettleMotionCommand_<ContainerAllocator>, 1, typename ContainerAllocator::template rebind<rj_msgs::msg::SettleMotionCommand_<ContainerAllocator>>::other> & _arg)
  {
    this->settle_command = _arg;
    return *this;
  }
  Type & set__collect_command(
    const rosidl_runtime_cpp::BoundedVector<rj_msgs::msg::CollectMotionCommand_<ContainerAllocator>, 1, typename ContainerAllocator::template rebind<rj_msgs::msg::CollectMotionCommand_<ContainerAllocator>>::other> & _arg)
  {
    this->collect_command = _arg;
    return *this;
  }
  Type & set__line_kick_command(
    const rosidl_runtime_cpp::BoundedVector<rj_msgs::msg::LineKickMotionCommand_<ContainerAllocator>, 1, typename ContainerAllocator::template rebind<rj_msgs::msg::LineKickMotionCommand_<ContainerAllocator>>::other> & _arg)
  {
    this->line_kick_command = _arg;
    return *this;
  }
  Type & set__intercept_command(
    const rosidl_runtime_cpp::BoundedVector<rj_msgs::msg::InterceptMotionCommand_<ContainerAllocator>, 1, typename ContainerAllocator::template rebind<rj_msgs::msg::InterceptMotionCommand_<ContainerAllocator>>::other> & _arg)
  {
    this->intercept_command = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    rj_msgs::msg::MotionCommand_<ContainerAllocator> *;
  using ConstRawPtr =
    const rj_msgs::msg::MotionCommand_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rj_msgs::msg::MotionCommand_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rj_msgs::msg::MotionCommand_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rj_msgs::msg::MotionCommand_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rj_msgs::msg::MotionCommand_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rj_msgs::msg::MotionCommand_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rj_msgs::msg::MotionCommand_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rj_msgs::msg::MotionCommand_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rj_msgs::msg::MotionCommand_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rj_msgs__msg__MotionCommand
    std::shared_ptr<rj_msgs::msg::MotionCommand_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rj_msgs__msg__MotionCommand
    std::shared_ptr<rj_msgs::msg::MotionCommand_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MotionCommand_ & other) const
  {
    if (this->empty_command != other.empty_command) {
      return false;
    }
    if (this->path_target_command != other.path_target_command) {
      return false;
    }
    if (this->world_vel_command != other.world_vel_command) {
      return false;
    }
    if (this->pivot_command != other.pivot_command) {
      return false;
    }
    if (this->settle_command != other.settle_command) {
      return false;
    }
    if (this->collect_command != other.collect_command) {
      return false;
    }
    if (this->line_kick_command != other.line_kick_command) {
      return false;
    }
    if (this->intercept_command != other.intercept_command) {
      return false;
    }
    return true;
  }
  bool operator!=(const MotionCommand_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MotionCommand_

// alias to use template instance with default allocator
using MotionCommand =
  rj_msgs::msg::MotionCommand_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace rj_msgs

#endif  // RJ_MSGS__MSG__DETAIL__MOTION_COMMAND__STRUCT_HPP_
