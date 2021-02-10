// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from rj_msgs:msg/ManipulatorSetpoint.idl
// generated code does not contain a copyright notice

#ifndef RJ_MSGS__MSG__DETAIL__MANIPULATOR_SETPOINT__STRUCT_HPP_
#define RJ_MSGS__MSG__DETAIL__MANIPULATOR_SETPOINT__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__rj_msgs__msg__ManipulatorSetpoint __attribute__((deprecated))
#else
# define DEPRECATED__rj_msgs__msg__ManipulatorSetpoint __declspec(deprecated)
#endif

namespace rj_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ManipulatorSetpoint_
{
  using Type = ManipulatorSetpoint_<ContainerAllocator>;

  explicit ManipulatorSetpoint_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->shoot_mode = 0;
      this->trigger_mode = 0;
      this->kick_strength = 0;
      this->dribbler_speed = 0.0f;
    }
  }

  explicit ManipulatorSetpoint_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->shoot_mode = 0;
      this->trigger_mode = 0;
      this->kick_strength = 0;
      this->dribbler_speed = 0.0f;
    }
  }

  // field types and members
  using _shoot_mode_type =
    uint8_t;
  _shoot_mode_type shoot_mode;
  using _trigger_mode_type =
    uint8_t;
  _trigger_mode_type trigger_mode;
  using _kick_strength_type =
    int8_t;
  _kick_strength_type kick_strength;
  using _dribbler_speed_type =
    float;
  _dribbler_speed_type dribbler_speed;

  // setters for named parameter idiom
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
  Type & set__kick_strength(
    const int8_t & _arg)
  {
    this->kick_strength = _arg;
    return *this;
  }
  Type & set__dribbler_speed(
    const float & _arg)
  {
    this->dribbler_speed = _arg;
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
    rj_msgs::msg::ManipulatorSetpoint_<ContainerAllocator> *;
  using ConstRawPtr =
    const rj_msgs::msg::ManipulatorSetpoint_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rj_msgs::msg::ManipulatorSetpoint_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rj_msgs::msg::ManipulatorSetpoint_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rj_msgs::msg::ManipulatorSetpoint_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rj_msgs::msg::ManipulatorSetpoint_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rj_msgs::msg::ManipulatorSetpoint_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rj_msgs::msg::ManipulatorSetpoint_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rj_msgs::msg::ManipulatorSetpoint_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rj_msgs::msg::ManipulatorSetpoint_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rj_msgs__msg__ManipulatorSetpoint
    std::shared_ptr<rj_msgs::msg::ManipulatorSetpoint_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rj_msgs__msg__ManipulatorSetpoint
    std::shared_ptr<rj_msgs::msg::ManipulatorSetpoint_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ManipulatorSetpoint_ & other) const
  {
    if (this->shoot_mode != other.shoot_mode) {
      return false;
    }
    if (this->trigger_mode != other.trigger_mode) {
      return false;
    }
    if (this->kick_strength != other.kick_strength) {
      return false;
    }
    if (this->dribbler_speed != other.dribbler_speed) {
      return false;
    }
    return true;
  }
  bool operator!=(const ManipulatorSetpoint_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ManipulatorSetpoint_

// alias to use template instance with default allocator
using ManipulatorSetpoint =
  rj_msgs::msg::ManipulatorSetpoint_<std::allocator<void>>;

// constant definitions
template<typename ContainerAllocator>
constexpr uint8_t ManipulatorSetpoint_<ContainerAllocator>::SHOOT_MODE_KICK;
template<typename ContainerAllocator>
constexpr uint8_t ManipulatorSetpoint_<ContainerAllocator>::SHOOT_MODE_CHIP;
template<typename ContainerAllocator>
constexpr uint8_t ManipulatorSetpoint_<ContainerAllocator>::TRIGGER_MODE_STAND_DOWN;
template<typename ContainerAllocator>
constexpr uint8_t ManipulatorSetpoint_<ContainerAllocator>::TRIGGER_MODE_IMMEDIATE;
template<typename ContainerAllocator>
constexpr uint8_t ManipulatorSetpoint_<ContainerAllocator>::TRIGGER_MODE_ON_BREAK_BEAM;

}  // namespace msg

}  // namespace rj_msgs

#endif  // RJ_MSGS__MSG__DETAIL__MANIPULATOR_SETPOINT__STRUCT_HPP_
