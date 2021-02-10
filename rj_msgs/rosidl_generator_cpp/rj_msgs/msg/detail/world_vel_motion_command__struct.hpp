// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from rj_msgs:msg/WorldVelMotionCommand.idl
// generated code does not contain a copyright notice

#ifndef RJ_MSGS__MSG__DETAIL__WORLD_VEL_MOTION_COMMAND__STRUCT_HPP_
#define RJ_MSGS__MSG__DETAIL__WORLD_VEL_MOTION_COMMAND__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'world_vel'
#include "rj_geometry_msgs/msg/detail/point__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__rj_msgs__msg__WorldVelMotionCommand __attribute__((deprecated))
#else
# define DEPRECATED__rj_msgs__msg__WorldVelMotionCommand __declspec(deprecated)
#endif

namespace rj_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct WorldVelMotionCommand_
{
  using Type = WorldVelMotionCommand_<ContainerAllocator>;

  explicit WorldVelMotionCommand_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : world_vel(_init)
  {
    (void)_init;
  }

  explicit WorldVelMotionCommand_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : world_vel(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _world_vel_type =
    rj_geometry_msgs::msg::Point_<ContainerAllocator>;
  _world_vel_type world_vel;

  // setters for named parameter idiom
  Type & set__world_vel(
    const rj_geometry_msgs::msg::Point_<ContainerAllocator> & _arg)
  {
    this->world_vel = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    rj_msgs::msg::WorldVelMotionCommand_<ContainerAllocator> *;
  using ConstRawPtr =
    const rj_msgs::msg::WorldVelMotionCommand_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rj_msgs::msg::WorldVelMotionCommand_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rj_msgs::msg::WorldVelMotionCommand_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rj_msgs::msg::WorldVelMotionCommand_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rj_msgs::msg::WorldVelMotionCommand_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rj_msgs::msg::WorldVelMotionCommand_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rj_msgs::msg::WorldVelMotionCommand_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rj_msgs::msg::WorldVelMotionCommand_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rj_msgs::msg::WorldVelMotionCommand_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rj_msgs__msg__WorldVelMotionCommand
    std::shared_ptr<rj_msgs::msg::WorldVelMotionCommand_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rj_msgs__msg__WorldVelMotionCommand
    std::shared_ptr<rj_msgs::msg::WorldVelMotionCommand_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const WorldVelMotionCommand_ & other) const
  {
    if (this->world_vel != other.world_vel) {
      return false;
    }
    return true;
  }
  bool operator!=(const WorldVelMotionCommand_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct WorldVelMotionCommand_

// alias to use template instance with default allocator
using WorldVelMotionCommand =
  rj_msgs::msg::WorldVelMotionCommand_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace rj_msgs

#endif  // RJ_MSGS__MSG__DETAIL__WORLD_VEL_MOTION_COMMAND__STRUCT_HPP_
