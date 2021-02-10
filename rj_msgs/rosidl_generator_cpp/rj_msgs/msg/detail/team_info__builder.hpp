// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rj_msgs:msg/TeamInfo.idl
// generated code does not contain a copyright notice

#ifndef RJ_MSGS__MSG__DETAIL__TEAM_INFO__BUILDER_HPP_
#define RJ_MSGS__MSG__DETAIL__TEAM_INFO__BUILDER_HPP_

#include "rj_msgs/msg/detail/team_info__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace rj_msgs
{

namespace msg
{

namespace builder
{

class Init_TeamInfo_goalie_id
{
public:
  explicit Init_TeamInfo_goalie_id(::rj_msgs::msg::TeamInfo & msg)
  : msg_(msg)
  {}
  ::rj_msgs::msg::TeamInfo goalie_id(::rj_msgs::msg::TeamInfo::_goalie_id_type arg)
  {
    msg_.goalie_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rj_msgs::msg::TeamInfo msg_;
};

class Init_TeamInfo_remaining_timeout_time
{
public:
  explicit Init_TeamInfo_remaining_timeout_time(::rj_msgs::msg::TeamInfo & msg)
  : msg_(msg)
  {}
  Init_TeamInfo_goalie_id remaining_timeout_time(::rj_msgs::msg::TeamInfo::_remaining_timeout_time_type arg)
  {
    msg_.remaining_timeout_time = std::move(arg);
    return Init_TeamInfo_goalie_id(msg_);
  }

private:
  ::rj_msgs::msg::TeamInfo msg_;
};

class Init_TeamInfo_timeouts_left
{
public:
  explicit Init_TeamInfo_timeouts_left(::rj_msgs::msg::TeamInfo & msg)
  : msg_(msg)
  {}
  Init_TeamInfo_remaining_timeout_time timeouts_left(::rj_msgs::msg::TeamInfo::_timeouts_left_type arg)
  {
    msg_.timeouts_left = std::move(arg);
    return Init_TeamInfo_remaining_timeout_time(msg_);
  }

private:
  ::rj_msgs::msg::TeamInfo msg_;
};

class Init_TeamInfo_yellow_card_remaining_times
{
public:
  explicit Init_TeamInfo_yellow_card_remaining_times(::rj_msgs::msg::TeamInfo & msg)
  : msg_(msg)
  {}
  Init_TeamInfo_timeouts_left yellow_card_remaining_times(::rj_msgs::msg::TeamInfo::_yellow_card_remaining_times_type arg)
  {
    msg_.yellow_card_remaining_times = std::move(arg);
    return Init_TeamInfo_timeouts_left(msg_);
  }

private:
  ::rj_msgs::msg::TeamInfo msg_;
};

class Init_TeamInfo_num_yellow_cards
{
public:
  explicit Init_TeamInfo_num_yellow_cards(::rj_msgs::msg::TeamInfo & msg)
  : msg_(msg)
  {}
  Init_TeamInfo_yellow_card_remaining_times num_yellow_cards(::rj_msgs::msg::TeamInfo::_num_yellow_cards_type arg)
  {
    msg_.num_yellow_cards = std::move(arg);
    return Init_TeamInfo_yellow_card_remaining_times(msg_);
  }

private:
  ::rj_msgs::msg::TeamInfo msg_;
};

class Init_TeamInfo_num_red_cards
{
public:
  explicit Init_TeamInfo_num_red_cards(::rj_msgs::msg::TeamInfo & msg)
  : msg_(msg)
  {}
  Init_TeamInfo_num_yellow_cards num_red_cards(::rj_msgs::msg::TeamInfo::_num_red_cards_type arg)
  {
    msg_.num_red_cards = std::move(arg);
    return Init_TeamInfo_num_yellow_cards(msg_);
  }

private:
  ::rj_msgs::msg::TeamInfo msg_;
};

class Init_TeamInfo_score
{
public:
  explicit Init_TeamInfo_score(::rj_msgs::msg::TeamInfo & msg)
  : msg_(msg)
  {}
  Init_TeamInfo_num_red_cards score(::rj_msgs::msg::TeamInfo::_score_type arg)
  {
    msg_.score = std::move(arg);
    return Init_TeamInfo_num_red_cards(msg_);
  }

private:
  ::rj_msgs::msg::TeamInfo msg_;
};

class Init_TeamInfo_name
{
public:
  Init_TeamInfo_name()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TeamInfo_score name(::rj_msgs::msg::TeamInfo::_name_type arg)
  {
    msg_.name = std::move(arg);
    return Init_TeamInfo_score(msg_);
  }

private:
  ::rj_msgs::msg::TeamInfo msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::rj_msgs::msg::TeamInfo>()
{
  return rj_msgs::msg::builder::Init_TeamInfo_name();
}

}  // namespace rj_msgs

#endif  // RJ_MSGS__MSG__DETAIL__TEAM_INFO__BUILDER_HPP_
