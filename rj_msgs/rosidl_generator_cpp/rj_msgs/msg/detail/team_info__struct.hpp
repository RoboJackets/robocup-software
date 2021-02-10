// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from rj_msgs:msg/TeamInfo.idl
// generated code does not contain a copyright notice

#ifndef RJ_MSGS__MSG__DETAIL__TEAM_INFO__STRUCT_HPP_
#define RJ_MSGS__MSG__DETAIL__TEAM_INFO__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'yellow_card_remaining_times'
// Member 'remaining_timeout_time'
#include "builtin_interfaces/msg/detail/duration__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__rj_msgs__msg__TeamInfo __attribute__((deprecated))
#else
# define DEPRECATED__rj_msgs__msg__TeamInfo __declspec(deprecated)
#endif

namespace rj_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct TeamInfo_
{
  using Type = TeamInfo_<ContainerAllocator>;

  explicit TeamInfo_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : remaining_timeout_time(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->name = "";
      this->score = 0l;
      this->num_red_cards = 0ull;
      this->num_yellow_cards = 0ull;
      this->timeouts_left = 0ull;
      this->goalie_id = 0;
    }
  }

  explicit TeamInfo_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : name(_alloc),
    remaining_timeout_time(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->name = "";
      this->score = 0l;
      this->num_red_cards = 0ull;
      this->num_yellow_cards = 0ull;
      this->timeouts_left = 0ull;
      this->goalie_id = 0;
    }
  }

  // field types and members
  using _name_type =
    std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>;
  _name_type name;
  using _score_type =
    int32_t;
  _score_type score;
  using _num_red_cards_type =
    uint64_t;
  _num_red_cards_type num_red_cards;
  using _num_yellow_cards_type =
    uint64_t;
  _num_yellow_cards_type num_yellow_cards;
  using _yellow_card_remaining_times_type =
    std::vector<builtin_interfaces::msg::Duration_<ContainerAllocator>, typename ContainerAllocator::template rebind<builtin_interfaces::msg::Duration_<ContainerAllocator>>::other>;
  _yellow_card_remaining_times_type yellow_card_remaining_times;
  using _timeouts_left_type =
    uint64_t;
  _timeouts_left_type timeouts_left;
  using _remaining_timeout_time_type =
    builtin_interfaces::msg::Duration_<ContainerAllocator>;
  _remaining_timeout_time_type remaining_timeout_time;
  using _goalie_id_type =
    uint8_t;
  _goalie_id_type goalie_id;

  // setters for named parameter idiom
  Type & set__name(
    const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other> & _arg)
  {
    this->name = _arg;
    return *this;
  }
  Type & set__score(
    const int32_t & _arg)
  {
    this->score = _arg;
    return *this;
  }
  Type & set__num_red_cards(
    const uint64_t & _arg)
  {
    this->num_red_cards = _arg;
    return *this;
  }
  Type & set__num_yellow_cards(
    const uint64_t & _arg)
  {
    this->num_yellow_cards = _arg;
    return *this;
  }
  Type & set__yellow_card_remaining_times(
    const std::vector<builtin_interfaces::msg::Duration_<ContainerAllocator>, typename ContainerAllocator::template rebind<builtin_interfaces::msg::Duration_<ContainerAllocator>>::other> & _arg)
  {
    this->yellow_card_remaining_times = _arg;
    return *this;
  }
  Type & set__timeouts_left(
    const uint64_t & _arg)
  {
    this->timeouts_left = _arg;
    return *this;
  }
  Type & set__remaining_timeout_time(
    const builtin_interfaces::msg::Duration_<ContainerAllocator> & _arg)
  {
    this->remaining_timeout_time = _arg;
    return *this;
  }
  Type & set__goalie_id(
    const uint8_t & _arg)
  {
    this->goalie_id = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    rj_msgs::msg::TeamInfo_<ContainerAllocator> *;
  using ConstRawPtr =
    const rj_msgs::msg::TeamInfo_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rj_msgs::msg::TeamInfo_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rj_msgs::msg::TeamInfo_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rj_msgs::msg::TeamInfo_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rj_msgs::msg::TeamInfo_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rj_msgs::msg::TeamInfo_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rj_msgs::msg::TeamInfo_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rj_msgs::msg::TeamInfo_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rj_msgs::msg::TeamInfo_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rj_msgs__msg__TeamInfo
    std::shared_ptr<rj_msgs::msg::TeamInfo_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rj_msgs__msg__TeamInfo
    std::shared_ptr<rj_msgs::msg::TeamInfo_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TeamInfo_ & other) const
  {
    if (this->name != other.name) {
      return false;
    }
    if (this->score != other.score) {
      return false;
    }
    if (this->num_red_cards != other.num_red_cards) {
      return false;
    }
    if (this->num_yellow_cards != other.num_yellow_cards) {
      return false;
    }
    if (this->yellow_card_remaining_times != other.yellow_card_remaining_times) {
      return false;
    }
    if (this->timeouts_left != other.timeouts_left) {
      return false;
    }
    if (this->remaining_timeout_time != other.remaining_timeout_time) {
      return false;
    }
    if (this->goalie_id != other.goalie_id) {
      return false;
    }
    return true;
  }
  bool operator!=(const TeamInfo_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TeamInfo_

// alias to use template instance with default allocator
using TeamInfo =
  rj_msgs::msg::TeamInfo_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace rj_msgs

#endif  // RJ_MSGS__MSG__DETAIL__TEAM_INFO__STRUCT_HPP_
