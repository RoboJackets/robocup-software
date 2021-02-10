// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from rj_msgs:msg/GameState.idl
// generated code does not contain a copyright notice

#ifndef RJ_MSGS__MSG__DETAIL__GAME_STATE__STRUCT_HPP_
#define RJ_MSGS__MSG__DETAIL__GAME_STATE__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'stage_time_left'
#include "builtin_interfaces/msg/detail/duration__struct.hpp"
// Member 'placement_point'
#include "rj_geometry_msgs/msg/detail/point__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__rj_msgs__msg__GameState __attribute__((deprecated))
#else
# define DEPRECATED__rj_msgs__msg__GameState __declspec(deprecated)
#endif

namespace rj_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct GameState_
{
  using Type = GameState_<ContainerAllocator>;

  explicit GameState_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stage_time_left(_init),
    placement_point(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->period = 0;
      this->state = 0;
      this->restart = 0;
      this->our_restart = false;
    }
  }

  explicit GameState_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stage_time_left(_alloc, _init),
    placement_point(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->period = 0;
      this->state = 0;
      this->restart = 0;
      this->our_restart = false;
    }
  }

  // field types and members
  using _period_type =
    uint8_t;
  _period_type period;
  using _state_type =
    uint8_t;
  _state_type state;
  using _restart_type =
    uint8_t;
  _restart_type restart;
  using _our_restart_type =
    bool;
  _our_restart_type our_restart;
  using _stage_time_left_type =
    builtin_interfaces::msg::Duration_<ContainerAllocator>;
  _stage_time_left_type stage_time_left;
  using _placement_point_type =
    rj_geometry_msgs::msg::Point_<ContainerAllocator>;
  _placement_point_type placement_point;

  // setters for named parameter idiom
  Type & set__period(
    const uint8_t & _arg)
  {
    this->period = _arg;
    return *this;
  }
  Type & set__state(
    const uint8_t & _arg)
  {
    this->state = _arg;
    return *this;
  }
  Type & set__restart(
    const uint8_t & _arg)
  {
    this->restart = _arg;
    return *this;
  }
  Type & set__our_restart(
    const bool & _arg)
  {
    this->our_restart = _arg;
    return *this;
  }
  Type & set__stage_time_left(
    const builtin_interfaces::msg::Duration_<ContainerAllocator> & _arg)
  {
    this->stage_time_left = _arg;
    return *this;
  }
  Type & set__placement_point(
    const rj_geometry_msgs::msg::Point_<ContainerAllocator> & _arg)
  {
    this->placement_point = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    rj_msgs::msg::GameState_<ContainerAllocator> *;
  using ConstRawPtr =
    const rj_msgs::msg::GameState_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rj_msgs::msg::GameState_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rj_msgs::msg::GameState_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rj_msgs::msg::GameState_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rj_msgs::msg::GameState_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rj_msgs::msg::GameState_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rj_msgs::msg::GameState_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rj_msgs::msg::GameState_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rj_msgs::msg::GameState_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rj_msgs__msg__GameState
    std::shared_ptr<rj_msgs::msg::GameState_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rj_msgs__msg__GameState
    std::shared_ptr<rj_msgs::msg::GameState_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GameState_ & other) const
  {
    if (this->period != other.period) {
      return false;
    }
    if (this->state != other.state) {
      return false;
    }
    if (this->restart != other.restart) {
      return false;
    }
    if (this->our_restart != other.our_restart) {
      return false;
    }
    if (this->stage_time_left != other.stage_time_left) {
      return false;
    }
    if (this->placement_point != other.placement_point) {
      return false;
    }
    return true;
  }
  bool operator!=(const GameState_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GameState_

// alias to use template instance with default allocator
using GameState =
  rj_msgs::msg::GameState_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace rj_msgs

#endif  // RJ_MSGS__MSG__DETAIL__GAME_STATE__STRUCT_HPP_
