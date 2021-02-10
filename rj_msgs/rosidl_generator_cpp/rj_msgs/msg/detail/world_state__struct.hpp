// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from rj_msgs:msg/WorldState.idl
// generated code does not contain a copyright notice

#ifndef RJ_MSGS__MSG__DETAIL__WORLD_STATE__STRUCT_HPP_
#define RJ_MSGS__MSG__DETAIL__WORLD_STATE__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'last_update_time'
#include "builtin_interfaces/msg/detail/time__struct.hpp"
// Member 'their_robots'
// Member 'our_robots'
#include "rj_msgs/msg/detail/robot_state__struct.hpp"
// Member 'ball'
#include "rj_msgs/msg/detail/ball_state__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__rj_msgs__msg__WorldState __attribute__((deprecated))
#else
# define DEPRECATED__rj_msgs__msg__WorldState __declspec(deprecated)
#endif

namespace rj_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct WorldState_
{
  using Type = WorldState_<ContainerAllocator>;

  explicit WorldState_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : last_update_time(_init),
    ball(_init)
  {
    (void)_init;
  }

  explicit WorldState_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : last_update_time(_alloc, _init),
    ball(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _last_update_time_type =
    builtin_interfaces::msg::Time_<ContainerAllocator>;
  _last_update_time_type last_update_time;
  using _their_robots_type =
    std::vector<rj_msgs::msg::RobotState_<ContainerAllocator>, typename ContainerAllocator::template rebind<rj_msgs::msg::RobotState_<ContainerAllocator>>::other>;
  _their_robots_type their_robots;
  using _our_robots_type =
    std::vector<rj_msgs::msg::RobotState_<ContainerAllocator>, typename ContainerAllocator::template rebind<rj_msgs::msg::RobotState_<ContainerAllocator>>::other>;
  _our_robots_type our_robots;
  using _ball_type =
    rj_msgs::msg::BallState_<ContainerAllocator>;
  _ball_type ball;

  // setters for named parameter idiom
  Type & set__last_update_time(
    const builtin_interfaces::msg::Time_<ContainerAllocator> & _arg)
  {
    this->last_update_time = _arg;
    return *this;
  }
  Type & set__their_robots(
    const std::vector<rj_msgs::msg::RobotState_<ContainerAllocator>, typename ContainerAllocator::template rebind<rj_msgs::msg::RobotState_<ContainerAllocator>>::other> & _arg)
  {
    this->their_robots = _arg;
    return *this;
  }
  Type & set__our_robots(
    const std::vector<rj_msgs::msg::RobotState_<ContainerAllocator>, typename ContainerAllocator::template rebind<rj_msgs::msg::RobotState_<ContainerAllocator>>::other> & _arg)
  {
    this->our_robots = _arg;
    return *this;
  }
  Type & set__ball(
    const rj_msgs::msg::BallState_<ContainerAllocator> & _arg)
  {
    this->ball = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    rj_msgs::msg::WorldState_<ContainerAllocator> *;
  using ConstRawPtr =
    const rj_msgs::msg::WorldState_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rj_msgs::msg::WorldState_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rj_msgs::msg::WorldState_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rj_msgs::msg::WorldState_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rj_msgs::msg::WorldState_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rj_msgs::msg::WorldState_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rj_msgs::msg::WorldState_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rj_msgs::msg::WorldState_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rj_msgs::msg::WorldState_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rj_msgs__msg__WorldState
    std::shared_ptr<rj_msgs::msg::WorldState_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rj_msgs__msg__WorldState
    std::shared_ptr<rj_msgs::msg::WorldState_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const WorldState_ & other) const
  {
    if (this->last_update_time != other.last_update_time) {
      return false;
    }
    if (this->their_robots != other.their_robots) {
      return false;
    }
    if (this->our_robots != other.our_robots) {
      return false;
    }
    if (this->ball != other.ball) {
      return false;
    }
    return true;
  }
  bool operator!=(const WorldState_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct WorldState_

// alias to use template instance with default allocator
using WorldState =
  rj_msgs::msg::WorldState_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace rj_msgs

#endif  // RJ_MSGS__MSG__DETAIL__WORLD_STATE__STRUCT_HPP_
