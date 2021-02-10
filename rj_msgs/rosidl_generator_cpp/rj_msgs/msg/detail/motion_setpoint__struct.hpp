// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from rj_msgs:msg/MotionSetpoint.idl
// generated code does not contain a copyright notice

#ifndef RJ_MSGS__MSG__DETAIL__MOTION_SETPOINT__STRUCT_HPP_
#define RJ_MSGS__MSG__DETAIL__MOTION_SETPOINT__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__rj_msgs__msg__MotionSetpoint __attribute__((deprecated))
#else
# define DEPRECATED__rj_msgs__msg__MotionSetpoint __declspec(deprecated)
#endif

namespace rj_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MotionSetpoint_
{
  using Type = MotionSetpoint_<ContainerAllocator>;

  explicit MotionSetpoint_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->velocity_x_mps = 0.0;
      this->velocity_y_mps = 0.0;
      this->velocity_z_radps = 0.0;
    }
  }

  explicit MotionSetpoint_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->velocity_x_mps = 0.0;
      this->velocity_y_mps = 0.0;
      this->velocity_z_radps = 0.0;
    }
  }

  // field types and members
  using _velocity_x_mps_type =
    double;
  _velocity_x_mps_type velocity_x_mps;
  using _velocity_y_mps_type =
    double;
  _velocity_y_mps_type velocity_y_mps;
  using _velocity_z_radps_type =
    double;
  _velocity_z_radps_type velocity_z_radps;

  // setters for named parameter idiom
  Type & set__velocity_x_mps(
    const double & _arg)
  {
    this->velocity_x_mps = _arg;
    return *this;
  }
  Type & set__velocity_y_mps(
    const double & _arg)
  {
    this->velocity_y_mps = _arg;
    return *this;
  }
  Type & set__velocity_z_radps(
    const double & _arg)
  {
    this->velocity_z_radps = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    rj_msgs::msg::MotionSetpoint_<ContainerAllocator> *;
  using ConstRawPtr =
    const rj_msgs::msg::MotionSetpoint_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rj_msgs::msg::MotionSetpoint_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rj_msgs::msg::MotionSetpoint_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rj_msgs::msg::MotionSetpoint_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rj_msgs::msg::MotionSetpoint_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rj_msgs::msg::MotionSetpoint_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rj_msgs::msg::MotionSetpoint_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rj_msgs::msg::MotionSetpoint_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rj_msgs::msg::MotionSetpoint_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rj_msgs__msg__MotionSetpoint
    std::shared_ptr<rj_msgs::msg::MotionSetpoint_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rj_msgs__msg__MotionSetpoint
    std::shared_ptr<rj_msgs::msg::MotionSetpoint_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MotionSetpoint_ & other) const
  {
    if (this->velocity_x_mps != other.velocity_x_mps) {
      return false;
    }
    if (this->velocity_y_mps != other.velocity_y_mps) {
      return false;
    }
    if (this->velocity_z_radps != other.velocity_z_radps) {
      return false;
    }
    return true;
  }
  bool operator!=(const MotionSetpoint_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MotionSetpoint_

// alias to use template instance with default allocator
using MotionSetpoint =
  rj_msgs::msg::MotionSetpoint_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace rj_msgs

#endif  // RJ_MSGS__MSG__DETAIL__MOTION_SETPOINT__STRUCT_HPP_
