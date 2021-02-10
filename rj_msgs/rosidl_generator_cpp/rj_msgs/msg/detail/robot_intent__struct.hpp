// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from rj_msgs:msg/RobotIntent.idl
// generated code does not contain a copyright notice

#ifndef RJ_MSGS__MSG__DETAIL__ROBOT_INTENT__STRUCT_HPP_
#define RJ_MSGS__MSG__DETAIL__ROBOT_INTENT__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'motion_command'
#include "rj_msgs/msg/detail/motion_command__struct.hpp"
// Member 'local_obstacles'
#include "rj_geometry_msgs/msg/detail/shape_set__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__rj_msgs__msg__RobotIntent __attribute__((deprecated))
#else
# define DEPRECATED__rj_msgs__msg__RobotIntent __declspec(deprecated)
#endif

namespace rj_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct RobotIntent_
{
  using Type = RobotIntent_<ContainerAllocator>;

  explicit RobotIntent_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : motion_command(_init),
    local_obstacles(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->shoot_mode = 0;
      this->trigger_mode = 0;
      this->kick_speed = 0.0f;
      this->dribbler_speed = 0.0f;
      this->is_active = false;
      this->priority = 0;
    }
  }

  explicit RobotIntent_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : motion_command(_alloc, _init),
    local_obstacles(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->shoot_mode = 0;
      this->trigger_mode = 0;
      this->kick_speed = 0.0f;
      this->dribbler_speed = 0.0f;
      this->is_active = false;
      this->priority = 0;
    }
  }

  // field types and members
  using _motion_command_type =
    rj_msgs::msg::MotionCommand_<ContainerAllocator>;
  _motion_command_type motion_command;
  using _local_obstacles_type =
    rj_geometry_msgs::msg::ShapeSet_<ContainerAllocator>;
  _local_obstacles_type local_obstacles;
  using _shoot_mode_type =
    uint8_t;
  _shoot_mode_type shoot_mode;
  using _trigger_mode_type =
    uint8_t;
  _trigger_mode_type trigger_mode;
  using _kick_speed_type =
    float;
  _kick_speed_type kick_speed;
  using _dribbler_speed_type =
    float;
  _dribbler_speed_type dribbler_speed;
  using _is_active_type =
    bool;
  _is_active_type is_active;
  using _priority_type =
    int8_t;
  _priority_type priority;

  // setters for named parameter idiom
  Type & set__motion_command(
    const rj_msgs::msg::MotionCommand_<ContainerAllocator> & _arg)
  {
    this->motion_command = _arg;
    return *this;
  }
  Type & set__local_obstacles(
    const rj_geometry_msgs::msg::ShapeSet_<ContainerAllocator> & _arg)
  {
    this->local_obstacles = _arg;
    return *this;
  }
  Type & set__shoot_mode(
    const uint8_t & _arg)
  {
    this->shoot_mode = _arg;
    return *this;
  }
  Type & set__trigger_mode(
    const uint8_t & _arg)
  {
    this->trigger_mode = _arg;
    return *this;
  }
  Type & set__kick_speed(
    const float & _arg)
  {
    this->kick_speed = _arg;
    return *this;
  }
  Type & set__dribbler_speed(
    const float & _arg)
  {
    this->dribbler_speed = _arg;
    return *this;
  }
  Type & set__is_active(
    const bool & _arg)
  {
    this->is_active = _arg;
    return *this;
  }
  Type & set__priority(
    const int8_t & _arg)
  {
    this->priority = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t SHOOT_MODE_KICK =
    0u;
  static constexpr uint8_t SHOOT_MODE_CHIP =
    1u;
  static constexpr uint8_t TRIGGER_MODE_STAND_DOWN =
    0u;
  static constexpr uint8_t TRIGGER_MODE_IMMEDIATE =
    1u;
  static constexpr uint8_t TRIGGER_MODE_ON_BREAK_BEAM =
    2u;

  // pointer types
  using RawPtr =
    rj_msgs::msg::RobotIntent_<ContainerAllocator> *;
  using ConstRawPtr =
    const rj_msgs::msg::RobotIntent_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rj_msgs::msg::RobotIntent_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rj_msgs::msg::RobotIntent_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rj_msgs::msg::RobotIntent_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rj_msgs::msg::RobotIntent_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rj_msgs::msg::RobotIntent_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rj_msgs::msg::RobotIntent_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rj_msgs::msg::RobotIntent_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rj_msgs::msg::RobotIntent_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rj_msgs__msg__RobotIntent
    std::shared_ptr<rj_msgs::msg::RobotIntent_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rj_msgs__msg__RobotIntent
    std::shared_ptr<rj_msgs::msg::RobotIntent_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RobotIntent_ & other) const
  {
    if (this->motion_command != other.motion_command) {
      return false;
    }
    if (this->local_obstacles != other.local_obstacles) {
      return false;
    }
    if (this->shoot_mode != other.shoot_mode) {
      return false;
    }
    if (this->trigger_mode != other.trigger_mode) {
      return false;
    }
    if (this->kick_speed != other.kick_speed) {
      return false;
    }
    if (this->dribbler_speed != other.dribbler_speed) {
      return false;
    }
    if (this->is_active != other.is_active) {
      return false;
    }
    if (this->priority != other.priority) {
      return false;
    }
    return true;
  }
  bool operator!=(const RobotIntent_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RobotIntent_

// alias to use template instance with default allocator
using RobotIntent =
  rj_msgs::msg::RobotIntent_<std::allocator<void>>;

// constant definitions
template<typename ContainerAllocator>
constexpr uint8_t RobotIntent_<ContainerAllocator>::SHOOT_MODE_KICK;
template<typename ContainerAllocator>
constexpr uint8_t RobotIntent_<ContainerAllocator>::SHOOT_MODE_CHIP;
template<typename ContainerAllocator>
constexpr uint8_t RobotIntent_<ContainerAllocator>::TRIGGER_MODE_STAND_DOWN;
template<typename ContainerAllocator>
constexpr uint8_t RobotIntent_<ContainerAllocator>::TRIGGER_MODE_IMMEDIATE;
template<typename ContainerAllocator>
constexpr uint8_t RobotIntent_<ContainerAllocator>::TRIGGER_MODE_ON_BREAK_BEAM;

}  // namespace msg

}  // namespace rj_msgs

#endif  // RJ_MSGS__MSG__DETAIL__ROBOT_INTENT__STRUCT_HPP_
