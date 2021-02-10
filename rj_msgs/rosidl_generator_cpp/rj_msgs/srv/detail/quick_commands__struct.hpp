// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from rj_msgs:srv/QuickCommands.idl
// generated code does not contain a copyright notice

#ifndef RJ_MSGS__SRV__DETAIL__QUICK_COMMANDS__STRUCT_HPP_
#define RJ_MSGS__SRV__DETAIL__QUICK_COMMANDS__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__rj_msgs__srv__QuickCommands_Request __attribute__((deprecated))
#else
# define DEPRECATED__rj_msgs__srv__QuickCommands_Request __declspec(deprecated)
#endif

namespace rj_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct QuickCommands_Request_
{
  using Type = QuickCommands_Request_<ContainerAllocator>;

  explicit QuickCommands_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->state = 0;
    }
  }

  explicit QuickCommands_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->state = 0;
    }
  }

  // field types and members
  using _state_type =
    uint8_t;
  _state_type state;

  // setters for named parameter idiom
  Type & set__state(
    const uint8_t & _arg)
  {
    this->state = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t COMMAND_HALT =
    0u;
  static constexpr uint8_t COMMAND_STOP =
    1u;
  static constexpr uint8_t COMMAND_READY =
    2u;
  static constexpr uint8_t COMMAND_PLAY =
    3u;

  // pointer types
  using RawPtr =
    rj_msgs::srv::QuickCommands_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const rj_msgs::srv::QuickCommands_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rj_msgs::srv::QuickCommands_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rj_msgs::srv::QuickCommands_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rj_msgs::srv::QuickCommands_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rj_msgs::srv::QuickCommands_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rj_msgs::srv::QuickCommands_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rj_msgs::srv::QuickCommands_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rj_msgs::srv::QuickCommands_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rj_msgs::srv::QuickCommands_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rj_msgs__srv__QuickCommands_Request
    std::shared_ptr<rj_msgs::srv::QuickCommands_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rj_msgs__srv__QuickCommands_Request
    std::shared_ptr<rj_msgs::srv::QuickCommands_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const QuickCommands_Request_ & other) const
  {
    if (this->state != other.state) {
      return false;
    }
    return true;
  }
  bool operator!=(const QuickCommands_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct QuickCommands_Request_

// alias to use template instance with default allocator
using QuickCommands_Request =
  rj_msgs::srv::QuickCommands_Request_<std::allocator<void>>;

// constant definitions
template<typename ContainerAllocator>
constexpr uint8_t QuickCommands_Request_<ContainerAllocator>::COMMAND_HALT;
template<typename ContainerAllocator>
constexpr uint8_t QuickCommands_Request_<ContainerAllocator>::COMMAND_STOP;
template<typename ContainerAllocator>
constexpr uint8_t QuickCommands_Request_<ContainerAllocator>::COMMAND_READY;
template<typename ContainerAllocator>
constexpr uint8_t QuickCommands_Request_<ContainerAllocator>::COMMAND_PLAY;

}  // namespace srv

}  // namespace rj_msgs


#ifndef _WIN32
# define DEPRECATED__rj_msgs__srv__QuickCommands_Response __attribute__((deprecated))
#else
# define DEPRECATED__rj_msgs__srv__QuickCommands_Response __declspec(deprecated)
#endif

namespace rj_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct QuickCommands_Response_
{
  using Type = QuickCommands_Response_<ContainerAllocator>;

  explicit QuickCommands_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  explicit QuickCommands_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    rj_msgs::srv::QuickCommands_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const rj_msgs::srv::QuickCommands_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rj_msgs::srv::QuickCommands_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rj_msgs::srv::QuickCommands_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rj_msgs::srv::QuickCommands_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rj_msgs::srv::QuickCommands_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rj_msgs::srv::QuickCommands_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rj_msgs::srv::QuickCommands_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rj_msgs::srv::QuickCommands_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rj_msgs::srv::QuickCommands_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rj_msgs__srv__QuickCommands_Response
    std::shared_ptr<rj_msgs::srv::QuickCommands_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rj_msgs__srv__QuickCommands_Response
    std::shared_ptr<rj_msgs::srv::QuickCommands_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const QuickCommands_Response_ & other) const
  {
    if (this->structure_needs_at_least_one_member != other.structure_needs_at_least_one_member) {
      return false;
    }
    return true;
  }
  bool operator!=(const QuickCommands_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct QuickCommands_Response_

// alias to use template instance with default allocator
using QuickCommands_Response =
  rj_msgs::srv::QuickCommands_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace rj_msgs

namespace rj_msgs
{

namespace srv
{

struct QuickCommands
{
  using Request = rj_msgs::srv::QuickCommands_Request;
  using Response = rj_msgs::srv::QuickCommands_Response;
};

}  // namespace srv

}  // namespace rj_msgs

#endif  // RJ_MSGS__SRV__DETAIL__QUICK_COMMANDS__STRUCT_HPP_
