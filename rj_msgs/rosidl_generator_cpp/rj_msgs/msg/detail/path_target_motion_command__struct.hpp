// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from rj_msgs:msg/PathTargetMotionCommand.idl
// generated code does not contain a copyright notice

#ifndef RJ_MSGS__MSG__DETAIL__PATH_TARGET_MOTION_COMMAND__STRUCT_HPP_
#define RJ_MSGS__MSG__DETAIL__PATH_TARGET_MOTION_COMMAND__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'target'
#include "rj_msgs/msg/detail/linear_motion_instant__struct.hpp"
// Member 'override_face_point'
#include "rj_geometry_msgs/msg/detail/point__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__rj_msgs__msg__PathTargetMotionCommand __attribute__((deprecated))
#else
# define DEPRECATED__rj_msgs__msg__PathTargetMotionCommand __declspec(deprecated)
#endif

namespace rj_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct PathTargetMotionCommand_
{
  using Type = PathTargetMotionCommand_<ContainerAllocator>;

  explicit PathTargetMotionCommand_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : target(_init)
  {
    (void)_init;
  }

  explicit PathTargetMotionCommand_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : target(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _target_type =
    rj_msgs::msg::LinearMotionInstant_<ContainerAllocator>;
  _target_type target;
  using _override_angle_type =
    rosidl_runtime_cpp::BoundedVector<double, 1, typename ContainerAllocator::template rebind<double>::other>;
  _override_angle_type override_angle;
  using _override_face_point_type =
    rosidl_runtime_cpp::BoundedVector<rj_geometry_msgs::msg::Point_<ContainerAllocator>, 1, typename ContainerAllocator::template rebind<rj_geometry_msgs::msg::Point_<ContainerAllocator>>::other>;
  _override_face_point_type override_face_point;

  // setters for named parameter idiom
  Type & set__target(
    const rj_msgs::msg::LinearMotionInstant_<ContainerAllocator> & _arg)
  {
    this->target = _arg;
    return *this;
  }
  Type & set__override_angle(
    const rosidl_runtime_cpp::BoundedVector<double, 1, typename ContainerAllocator::template rebind<double>::other> & _arg)
  {
    this->override_angle = _arg;
    return *this;
  }
  Type & set__override_face_point(
    const rosidl_runtime_cpp::BoundedVector<rj_geometry_msgs::msg::Point_<ContainerAllocator>, 1, typename ContainerAllocator::template rebind<rj_geometry_msgs::msg::Point_<ContainerAllocator>>::other> & _arg)
  {
    this->override_face_point = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    rj_msgs::msg::PathTargetMotionCommand_<ContainerAllocator> *;
  using ConstRawPtr =
    const rj_msgs::msg::PathTargetMotionCommand_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rj_msgs::msg::PathTargetMotionCommand_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rj_msgs::msg::PathTargetMotionCommand_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rj_msgs::msg::PathTargetMotionCommand_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rj_msgs::msg::PathTargetMotionCommand_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rj_msgs::msg::PathTargetMotionCommand_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rj_msgs::msg::PathTargetMotionCommand_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rj_msgs::msg::PathTargetMotionCommand_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rj_msgs::msg::PathTargetMotionCommand_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rj_msgs__msg__PathTargetMotionCommand
    std::shared_ptr<rj_msgs::msg::PathTargetMotionCommand_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rj_msgs__msg__PathTargetMotionCommand
    std::shared_ptr<rj_msgs::msg::PathTargetMotionCommand_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PathTargetMotionCommand_ & other) const
  {
    if (this->target != other.target) {
      return false;
    }
    if (this->override_angle != other.override_angle) {
      return false;
    }
    if (this->override_face_point != other.override_face_point) {
      return false;
    }
    return true;
  }
  bool operator!=(const PathTargetMotionCommand_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PathTargetMotionCommand_

// alias to use template instance with default allocator
using PathTargetMotionCommand =
  rj_msgs::msg::PathTargetMotionCommand_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace rj_msgs

#endif  // RJ_MSGS__MSG__DETAIL__PATH_TARGET_MOTION_COMMAND__STRUCT_HPP_
