// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from rj_msgs:srv/SetGameSettings.idl
// generated code does not contain a copyright notice

#ifndef RJ_MSGS__SRV__DETAIL__SET_GAME_SETTINGS__STRUCT_HPP_
#define RJ_MSGS__SRV__DETAIL__SET_GAME_SETTINGS__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'game_settings'
#include "rj_msgs/msg/detail/game_settings__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__rj_msgs__srv__SetGameSettings_Request __attribute__((deprecated))
#else
# define DEPRECATED__rj_msgs__srv__SetGameSettings_Request __declspec(deprecated)
#endif

namespace rj_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SetGameSettings_Request_
{
  using Type = SetGameSettings_Request_<ContainerAllocator>;

  explicit SetGameSettings_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : game_settings(_init)
  {
    (void)_init;
  }

  explicit SetGameSettings_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : game_settings(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _game_settings_type =
    rj_msgs::msg::GameSettings_<ContainerAllocator>;
  _game_settings_type game_settings;

  // setters for named parameter idiom
  Type & set__game_settings(
    const rj_msgs::msg::GameSettings_<ContainerAllocator> & _arg)
  {
    this->game_settings = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    rj_msgs::srv::SetGameSettings_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const rj_msgs::srv::SetGameSettings_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rj_msgs::srv::SetGameSettings_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rj_msgs::srv::SetGameSettings_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rj_msgs::srv::SetGameSettings_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rj_msgs::srv::SetGameSettings_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rj_msgs::srv::SetGameSettings_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rj_msgs::srv::SetGameSettings_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rj_msgs::srv::SetGameSettings_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rj_msgs::srv::SetGameSettings_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rj_msgs__srv__SetGameSettings_Request
    std::shared_ptr<rj_msgs::srv::SetGameSettings_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rj_msgs__srv__SetGameSettings_Request
    std::shared_ptr<rj_msgs::srv::SetGameSettings_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetGameSettings_Request_ & other) const
  {
    if (this->game_settings != other.game_settings) {
      return false;
    }
    return true;
  }
  bool operator!=(const SetGameSettings_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetGameSettings_Request_

// alias to use template instance with default allocator
using SetGameSettings_Request =
  rj_msgs::srv::SetGameSettings_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace rj_msgs


#ifndef _WIN32
# define DEPRECATED__rj_msgs__srv__SetGameSettings_Response __attribute__((deprecated))
#else
# define DEPRECATED__rj_msgs__srv__SetGameSettings_Response __declspec(deprecated)
#endif

namespace rj_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SetGameSettings_Response_
{
  using Type = SetGameSettings_Response_<ContainerAllocator>;

  explicit SetGameSettings_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  explicit SetGameSettings_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    rj_msgs::srv::SetGameSettings_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const rj_msgs::srv::SetGameSettings_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rj_msgs::srv::SetGameSettings_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rj_msgs::srv::SetGameSettings_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rj_msgs::srv::SetGameSettings_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rj_msgs::srv::SetGameSettings_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rj_msgs::srv::SetGameSettings_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rj_msgs::srv::SetGameSettings_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rj_msgs::srv::SetGameSettings_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rj_msgs::srv::SetGameSettings_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rj_msgs__srv__SetGameSettings_Response
    std::shared_ptr<rj_msgs::srv::SetGameSettings_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rj_msgs__srv__SetGameSettings_Response
    std::shared_ptr<rj_msgs::srv::SetGameSettings_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetGameSettings_Response_ & other) const
  {
    if (this->structure_needs_at_least_one_member != other.structure_needs_at_least_one_member) {
      return false;
    }
    return true;
  }
  bool operator!=(const SetGameSettings_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetGameSettings_Response_

// alias to use template instance with default allocator
using SetGameSettings_Response =
  rj_msgs::srv::SetGameSettings_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace rj_msgs

namespace rj_msgs
{

namespace srv
{

struct SetGameSettings
{
  using Request = rj_msgs::srv::SetGameSettings_Request;
  using Response = rj_msgs::srv::SetGameSettings_Response;
};

}  // namespace srv

}  // namespace rj_msgs

#endif  // RJ_MSGS__SRV__DETAIL__SET_GAME_SETTINGS__STRUCT_HPP_
