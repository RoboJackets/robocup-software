// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from rj_msgs:msg/SettleMotionCommand.idl
// generated code does not contain a copyright notice

#ifndef RJ_MSGS__MSG__DETAIL__SETTLE_MOTION_COMMAND__STRUCT_HPP_
#define RJ_MSGS__MSG__DETAIL__SETTLE_MOTION_COMMAND__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'maybe_target'
#include "rj_geometry_msgs/msg/detail/point__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__rj_msgs__msg__SettleMotionCommand __attribute__((deprecated))
#else
# define DEPRECATED__rj_msgs__msg__SettleMotionCommand __declspec(deprecated)
#endif

namespace rj_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct SettleMotionCommand_
{
  using Type = SettleMotionCommand_<ContainerAllocator>;

  explicit SettleMotionCommand_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit SettleMotionCommand_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _maybe_target_type =
    rosidl_runtime_cpp::BoundedVector<rj_geometry_msgs::msg::Point_<ContainerAllocator>, 1, typename ContainerAllocator::template rebind<rj_geometry_msgs::msg::Point_<ContainerAllocator>>::other>;
  _maybe_target_type maybe_target;

  // setters for named parameter idiom
  Type & set__maybe_target(
    const rosidl_runtime_cpp::BoundedVector<rj_geometry_msgs::msg::Point_<ContainerAllocator>, 1, typename ContainerAllocator::template rebind<rj_geometry_msgs::msg::Point_<ContainerAllocator>>::other> & _arg)
  {
    this->maybe_target = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    rj_msgs::msg::SettleMotionCommand_<ContainerAllocator> *;
  using ConstRawPtr =
    const rj_msgs::msg::SettleMotionCommand_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rj_msgs::msg::SettleMotionCommand_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rj_msgs::msg::SettleMotionCommand_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rj_msgs::msg::SettleMotionCommand_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rj_msgs::msg::SettleMotionCommand_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rj_msgs::msg::SettleMotionCommand_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rj_msgs::msg::SettleMotionCommand_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rj_msgs::msg::SettleMotionCommand_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rj_msgs::msg::SettleMotionCommand_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rj_msgs__msg__SettleMotionCommand
    std::shared_ptr<rj_msgs::msg::SettleMotionCommand_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rj_msgs__msg__SettleMotionCommand
    std::shared_ptr<rj_msgs::msg::SettleMotionCommand_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SettleMotionCommand_ & other) const
  {
    if (this->maybe_target != other.maybe_target) {
      return false;
    }
    return true;
  }
  bool operator!=(const SettleMotionCommand_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SettleMotionCommand_

// alias to use template instance with default allocator
using SettleMotionCommand =
  rj_msgs::msg::SettleMotionCommand_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace rj_msgs

#endif  // RJ_MSGS__MSG__DETAIL__SETTLE_MOTION_COMMAND__STRUCT_HPP_
