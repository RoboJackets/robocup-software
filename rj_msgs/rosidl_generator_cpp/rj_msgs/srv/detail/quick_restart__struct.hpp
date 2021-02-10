// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from rj_msgs:srv/QuickRestart.idl
// generated code does not contain a copyright notice

#ifndef RJ_MSGS__SRV__DETAIL__QUICK_RESTART__STRUCT_HPP_
#define RJ_MSGS__SRV__DETAIL__QUICK_RESTART__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__rj_msgs__srv__QuickRestart_Request __attribute__((deprecated))
#else
# define DEPRECATED__rj_msgs__srv__QuickRestart_Request __declspec(deprecated)
#endif

namespace rj_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct QuickRestart_Request_
{
  using Type = QuickRestart_Request_<ContainerAllocator>;

  explicit QuickRestart_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->restart = 0;
      this->blue_team = false;
    }
  }

  explicit QuickRestart_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->restart = 0;
      this->blue_team = false;
    }
  }

  // field types and members
  using _restart_type =
    uint8_t;
  _restart_type restart;
  using _blue_team_type =
    bool;
  _blue_team_type blue_team;

  // setters for named parameter idiom
  Type & set__restart(
    const uint8_t & _arg)
  {
    this->restart = _arg;
    return *this;
  }
  Type & set__blue_team(
    const bool & _arg)
  {
    this->blue_team = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t RESTART_KICKOFF =
    0u;
  static constexpr uint8_t RESTART_DIRECT =
    1u;
  static constexpr uint8_t RESTART_INDIRECT =
    2u;

  // pointer types
  using RawPtr =
    rj_msgs::srv::QuickRestart_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const rj_msgs::srv::QuickRestart_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rj_msgs::srv::QuickRestart_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rj_msgs::srv::QuickRestart_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rj_msgs::srv::QuickRestart_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rj_msgs::srv::QuickRestart_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rj_msgs::srv::QuickRestart_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rj_msgs::srv::QuickRestart_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rj_msgs::srv::QuickRestart_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rj_msgs::srv::QuickRestart_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rj_msgs__srv__QuickRestart_Request
    std::shared_ptr<rj_msgs::srv::QuickRestart_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rj_msgs__srv__QuickRestart_Request
    std::shared_ptr<rj_msgs::srv::QuickRestart_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const QuickRestart_Request_ & other) const
  {
    if (this->restart != other.restart) {
      return false;
    }
    if (this->blue_team != other.blue_team) {
      return false;
    }
    return true;
  }
  bool operator!=(const QuickRestart_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct QuickRestart_Request_

// alias to use template instance with default allocator
using QuickRestart_Request =
  rj_msgs::srv::QuickRestart_Request_<std::allocator<void>>;

// constant definitions
template<typename ContainerAllocator>
constexpr uint8_t QuickRestart_Request_<ContainerAllocator>::RESTART_KICKOFF;
template<typename ContainerAllocator>
constexpr uint8_t QuickRestart_Request_<ContainerAllocator>::RESTART_DIRECT;
template<typename ContainerAllocator>
constexpr uint8_t QuickRestart_Request_<ContainerAllocator>::RESTART_INDIRECT;

}  // namespace srv

}  // namespace rj_msgs


#ifndef _WIN32
# define DEPRECATED__rj_msgs__srv__QuickRestart_Response __attribute__((deprecated))
#else
# define DEPRECATED__rj_msgs__srv__QuickRestart_Response __declspec(deprecated)
#endif

namespace rj_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct QuickRestart_Response_
{
  using Type = QuickRestart_Response_<ContainerAllocator>;

  explicit QuickRestart_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  explicit QuickRestart_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    rj_msgs::srv::QuickRestart_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const rj_msgs::srv::QuickRestart_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rj_msgs::srv::QuickRestart_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rj_msgs::srv::QuickRestart_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rj_msgs::srv::QuickRestart_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rj_msgs::srv::QuickRestart_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rj_msgs::srv::QuickRestart_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rj_msgs::srv::QuickRestart_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rj_msgs::srv::QuickRestart_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rj_msgs::srv::QuickRestart_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rj_msgs__srv__QuickRestart_Response
    std::shared_ptr<rj_msgs::srv::QuickRestart_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rj_msgs__srv__QuickRestart_Response
    std::shared_ptr<rj_msgs::srv::QuickRestart_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const QuickRestart_Response_ & other) const
  {
    if (this->structure_needs_at_least_one_member != other.structure_needs_at_least_one_member) {
      return false;
    }
    return true;
  }
  bool operator!=(const QuickRestart_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct QuickRestart_Response_

// alias to use template instance with default allocator
using QuickRestart_Response =
  rj_msgs::srv::QuickRestart_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace rj_msgs

namespace rj_msgs
{

namespace srv
{

struct QuickRestart
{
  using Request = rj_msgs::srv::QuickRestart_Request;
  using Response = rj_msgs::srv::QuickRestart_Response;
};

}  // namespace srv

}  // namespace rj_msgs

#endif  // RJ_MSGS__SRV__DETAIL__QUICK_RESTART__STRUCT_HPP_
