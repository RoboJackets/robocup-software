// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from rj_msgs:msg/CollectMotionCommand.idl
// generated code does not contain a copyright notice

#ifndef RJ_MSGS__MSG__DETAIL__COLLECT_MOTION_COMMAND__STRUCT_HPP_
#define RJ_MSGS__MSG__DETAIL__COLLECT_MOTION_COMMAND__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__rj_msgs__msg__CollectMotionCommand __attribute__((deprecated))
#else
# define DEPRECATED__rj_msgs__msg__CollectMotionCommand __declspec(deprecated)
#endif

namespace rj_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct CollectMotionCommand_
{
  using Type = CollectMotionCommand_<ContainerAllocator>;

  explicit CollectMotionCommand_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  explicit CollectMotionCommand_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  // field types and members
  using _structure_needs_at_least_one_member_type =
    uint8_t;
  _structure_needs_at_least_one_member_type structure_needs_at_least_one_member;


  // constant declarations

  // pointer types
  using RawPtr =
    rj_msgs::msg::CollectMotionCommand_<ContainerAllocator> *;
  using ConstRawPtr =
    const rj_msgs::msg::CollectMotionCommand_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rj_msgs::msg::CollectMotionCommand_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rj_msgs::msg::CollectMotionCommand_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rj_msgs::msg::CollectMotionCommand_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rj_msgs::msg::CollectMotionCommand_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rj_msgs::msg::CollectMotionCommand_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rj_msgs::msg::CollectMotionCommand_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rj_msgs::msg::CollectMotionCommand_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rj_msgs::msg::CollectMotionCommand_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rj_msgs__msg__CollectMotionCommand
    std::shared_ptr<rj_msgs::msg::CollectMotionCommand_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rj_msgs__msg__CollectMotionCommand
    std::shared_ptr<rj_msgs::msg::CollectMotionCommand_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const CollectMotionCommand_ & other) const
  {
    if (this->structure_needs_at_least_one_member != other.structure_needs_at_least_one_member) {
      return false;
    }
    return true;
  }
  bool operator!=(const CollectMotionCommand_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct CollectMotionCommand_

// alias to use template instance with default allocator
using CollectMotionCommand =
  rj_msgs::msg::CollectMotionCommand_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace rj_msgs

#endif  // RJ_MSGS__MSG__DETAIL__COLLECT_MOTION_COMMAND__STRUCT_HPP_
