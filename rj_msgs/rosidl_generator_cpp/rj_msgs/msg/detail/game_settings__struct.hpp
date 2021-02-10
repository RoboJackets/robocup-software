// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from rj_msgs:msg/GameSettings.idl
// generated code does not contain a copyright notice

#ifndef RJ_MSGS__MSG__DETAIL__GAME_SETTINGS__STRUCT_HPP_
#define RJ_MSGS__MSG__DETAIL__GAME_SETTINGS__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__rj_msgs__msg__GameSettings __attribute__((deprecated))
#else
# define DEPRECATED__rj_msgs__msg__GameSettings __declspec(deprecated)
#endif

namespace rj_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct GameSettings_
{
  using Type = GameSettings_<ContainerAllocator>;

  explicit GameSettings_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->simulation = false;
      this->request_blue_team = false;
      this->request_goalie_id = 0l;
      this->defend_plus_x = false;
      this->use_our_half = false;
      this->use_their_half = false;
      this->paused = false;
    }
  }

  explicit GameSettings_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->simulation = false;
      this->request_blue_team = false;
      this->request_goalie_id = 0l;
      this->defend_plus_x = false;
      this->use_our_half = false;
      this->use_their_half = false;
      this->paused = false;
    }
  }

  // field types and members
  using _simulation_type =
    bool;
  _simulation_type simulation;
  using _request_blue_team_type =
    bool;
  _request_blue_team_type request_blue_team;
  using _request_goalie_id_type =
    int32_t;
  _request_goalie_id_type request_goalie_id;
  using _defend_plus_x_type =
    bool;
  _defend_plus_x_type defend_plus_x;
  using _use_our_half_type =
    bool;
  _use_our_half_type use_our_half;
  using _use_their_half_type =
    bool;
  _use_their_half_type use_their_half;
  using _paused_type =
    bool;
  _paused_type paused;

  // setters for named parameter idiom
  Type & set__simulation(
    const bool & _arg)
  {
    this->simulation = _arg;
    return *this;
  }
  Type & set__request_blue_team(
    const bool & _arg)
  {
    this->request_blue_team = _arg;
    return *this;
  }
  Type & set__request_goalie_id(
    const int32_t & _arg)
  {
    this->request_goalie_id = _arg;
    return *this;
  }
  Type & set__defend_plus_x(
    const bool & _arg)
  {
    this->defend_plus_x = _arg;
    return *this;
  }
  Type & set__use_our_half(
    const bool & _arg)
  {
    this->use_our_half = _arg;
    return *this;
  }
  Type & set__use_their_half(
    const bool & _arg)
  {
    this->use_their_half = _arg;
    return *this;
  }
  Type & set__paused(
    const bool & _arg)
  {
    this->paused = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    rj_msgs::msg::GameSettings_<ContainerAllocator> *;
  using ConstRawPtr =
    const rj_msgs::msg::GameSettings_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rj_msgs::msg::GameSettings_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rj_msgs::msg::GameSettings_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rj_msgs::msg::GameSettings_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rj_msgs::msg::GameSettings_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rj_msgs::msg::GameSettings_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rj_msgs::msg::GameSettings_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rj_msgs::msg::GameSettings_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rj_msgs::msg::GameSettings_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rj_msgs__msg__GameSettings
    std::shared_ptr<rj_msgs::msg::GameSettings_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rj_msgs__msg__GameSettings
    std::shared_ptr<rj_msgs::msg::GameSettings_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GameSettings_ & other) const
  {
    if (this->simulation != other.simulation) {
      return false;
    }
    if (this->request_blue_team != other.request_blue_team) {
      return false;
    }
    if (this->request_goalie_id != other.request_goalie_id) {
      return false;
    }
    if (this->defend_plus_x != other.defend_plus_x) {
      return false;
    }
    if (this->use_our_half != other.use_our_half) {
      return false;
    }
    if (this->use_their_half != other.use_their_half) {
      return false;
    }
    if (this->paused != other.paused) {
      return false;
    }
    return true;
  }
  bool operator!=(const GameSettings_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GameSettings_

// alias to use template instance with default allocator
using GameSettings =
  rj_msgs::msg::GameSettings_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace rj_msgs

#endif  // RJ_MSGS__MSG__DETAIL__GAME_SETTINGS__STRUCT_HPP_
