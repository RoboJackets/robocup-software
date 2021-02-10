// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from rj_msgs:msg/RobotStatus.idl
// generated code does not contain a copyright notice

#ifndef RJ_MSGS__MSG__DETAIL__ROBOT_STATUS__STRUCT_HPP_
#define RJ_MSGS__MSG__DETAIL__ROBOT_STATUS__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__rj_msgs__msg__RobotStatus __attribute__((deprecated))
#else
# define DEPRECATED__rj_msgs__msg__RobotStatus __declspec(deprecated)
#endif

namespace rj_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct RobotStatus_
{
  using Type = RobotStatus_<ContainerAllocator>;

  explicit RobotStatus_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->robot_id = 0;
      this->battery_voltage = 0.0;
      std::fill<typename std::array<bool, 5>::iterator, bool>(this->motor_errors.begin(), this->motor_errors.end(), false);
      this->has_ball_sense = false;
      this->kicker_charged = false;
      this->kicker_healthy = false;
      this->fpga_error = false;
      std::fill<typename std::array<int16_t, 4>::iterator, int16_t>(this->encoder_deltas.begin(), this->encoder_deltas.end(), 0);
    }
  }

  explicit RobotStatus_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : motor_errors(_alloc),
    encoder_deltas(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->robot_id = 0;
      this->battery_voltage = 0.0;
      std::fill<typename std::array<bool, 5>::iterator, bool>(this->motor_errors.begin(), this->motor_errors.end(), false);
      this->has_ball_sense = false;
      this->kicker_charged = false;
      this->kicker_healthy = false;
      this->fpga_error = false;
      std::fill<typename std::array<int16_t, 4>::iterator, int16_t>(this->encoder_deltas.begin(), this->encoder_deltas.end(), 0);
    }
  }

  // field types and members
  using _robot_id_type =
    uint8_t;
  _robot_id_type robot_id;
  using _battery_voltage_type =
    double;
  _battery_voltage_type battery_voltage;
  using _motor_errors_type =
    std::array<bool, 5>;
  _motor_errors_type motor_errors;
  using _has_ball_sense_type =
    bool;
  _has_ball_sense_type has_ball_sense;
  using _kicker_charged_type =
    bool;
  _kicker_charged_type kicker_charged;
  using _kicker_healthy_type =
    bool;
  _kicker_healthy_type kicker_healthy;
  using _fpga_error_type =
    bool;
  _fpga_error_type fpga_error;
  using _encoder_deltas_type =
    std::array<int16_t, 4>;
  _encoder_deltas_type encoder_deltas;

  // setters for named parameter idiom
  Type & set__robot_id(
    const uint8_t & _arg)
  {
    this->robot_id = _arg;
    return *this;
  }
  Type & set__battery_voltage(
    const double & _arg)
  {
    this->battery_voltage = _arg;
    return *this;
  }
  Type & set__motor_errors(
    const std::array<bool, 5> & _arg)
  {
    this->motor_errors = _arg;
    return *this;
  }
  Type & set__has_ball_sense(
    const bool & _arg)
  {
    this->has_ball_sense = _arg;
    return *this;
  }
  Type & set__kicker_charged(
    const bool & _arg)
  {
    this->kicker_charged = _arg;
    return *this;
  }
  Type & set__kicker_healthy(
    const bool & _arg)
  {
    this->kicker_healthy = _arg;
    return *this;
  }
  Type & set__fpga_error(
    const bool & _arg)
  {
    this->fpga_error = _arg;
    return *this;
  }
  Type & set__encoder_deltas(
    const std::array<int16_t, 4> & _arg)
  {
    this->encoder_deltas = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    rj_msgs::msg::RobotStatus_<ContainerAllocator> *;
  using ConstRawPtr =
    const rj_msgs::msg::RobotStatus_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rj_msgs::msg::RobotStatus_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rj_msgs::msg::RobotStatus_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rj_msgs::msg::RobotStatus_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rj_msgs::msg::RobotStatus_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rj_msgs::msg::RobotStatus_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rj_msgs::msg::RobotStatus_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rj_msgs::msg::RobotStatus_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rj_msgs::msg::RobotStatus_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rj_msgs__msg__RobotStatus
    std::shared_ptr<rj_msgs::msg::RobotStatus_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rj_msgs__msg__RobotStatus
    std::shared_ptr<rj_msgs::msg::RobotStatus_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RobotStatus_ & other) const
  {
    if (this->robot_id != other.robot_id) {
      return false;
    }
    if (this->battery_voltage != other.battery_voltage) {
      return false;
    }
    if (this->motor_errors != other.motor_errors) {
      return false;
    }
    if (this->has_ball_sense != other.has_ball_sense) {
      return false;
    }
    if (this->kicker_charged != other.kicker_charged) {
      return false;
    }
    if (this->kicker_healthy != other.kicker_healthy) {
      return false;
    }
    if (this->fpga_error != other.fpga_error) {
      return false;
    }
    if (this->encoder_deltas != other.encoder_deltas) {
      return false;
    }
    return true;
  }
  bool operator!=(const RobotStatus_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RobotStatus_

// alias to use template instance with default allocator
using RobotStatus =
  rj_msgs::msg::RobotStatus_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace rj_msgs

#endif  // RJ_MSGS__MSG__DETAIL__ROBOT_STATUS__STRUCT_HPP_
