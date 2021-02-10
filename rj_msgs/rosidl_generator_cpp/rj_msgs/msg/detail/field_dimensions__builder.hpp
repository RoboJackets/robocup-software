// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rj_msgs:msg/FieldDimensions.idl
// generated code does not contain a copyright notice

#ifndef RJ_MSGS__MSG__DETAIL__FIELD_DIMENSIONS__BUILDER_HPP_
#define RJ_MSGS__MSG__DETAIL__FIELD_DIMENSIONS__BUILDER_HPP_

#include "rj_msgs/msg/detail/field_dimensions__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace rj_msgs
{

namespace msg
{

namespace builder
{

class Init_FieldDimensions_floor_width
{
public:
  explicit Init_FieldDimensions_floor_width(::rj_msgs::msg::FieldDimensions & msg)
  : msg_(msg)
  {}
  ::rj_msgs::msg::FieldDimensions floor_width(::rj_msgs::msg::FieldDimensions::_floor_width_type arg)
  {
    msg_.floor_width = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rj_msgs::msg::FieldDimensions msg_;
};

class Init_FieldDimensions_floor_length
{
public:
  explicit Init_FieldDimensions_floor_length(::rj_msgs::msg::FieldDimensions & msg)
  : msg_(msg)
  {}
  Init_FieldDimensions_floor_width floor_length(::rj_msgs::msg::FieldDimensions::_floor_length_type arg)
  {
    msg_.floor_length = std::move(arg);
    return Init_FieldDimensions_floor_width(msg_);
  }

private:
  ::rj_msgs::msg::FieldDimensions msg_;
};

class Init_FieldDimensions_goal_flat
{
public:
  explicit Init_FieldDimensions_goal_flat(::rj_msgs::msg::FieldDimensions & msg)
  : msg_(msg)
  {}
  Init_FieldDimensions_floor_length goal_flat(::rj_msgs::msg::FieldDimensions::_goal_flat_type arg)
  {
    msg_.goal_flat = std::move(arg);
    return Init_FieldDimensions_floor_length(msg_);
  }

private:
  ::rj_msgs::msg::FieldDimensions msg_;
};

class Init_FieldDimensions_center_diameter
{
public:
  explicit Init_FieldDimensions_center_diameter(::rj_msgs::msg::FieldDimensions & msg)
  : msg_(msg)
  {}
  Init_FieldDimensions_goal_flat center_diameter(::rj_msgs::msg::FieldDimensions::_center_diameter_type arg)
  {
    msg_.center_diameter = std::move(arg);
    return Init_FieldDimensions_goal_flat(msg_);
  }

private:
  ::rj_msgs::msg::FieldDimensions msg_;
};

class Init_FieldDimensions_center_radius
{
public:
  explicit Init_FieldDimensions_center_radius(::rj_msgs::msg::FieldDimensions & msg)
  : msg_(msg)
  {}
  Init_FieldDimensions_center_diameter center_radius(::rj_msgs::msg::FieldDimensions::_center_radius_type arg)
  {
    msg_.center_radius = std::move(arg);
    return Init_FieldDimensions_center_diameter(msg_);
  }

private:
  ::rj_msgs::msg::FieldDimensions msg_;
};

class Init_FieldDimensions_penalty_long_dist
{
public:
  explicit Init_FieldDimensions_penalty_long_dist(::rj_msgs::msg::FieldDimensions & msg)
  : msg_(msg)
  {}
  Init_FieldDimensions_center_radius penalty_long_dist(::rj_msgs::msg::FieldDimensions::_penalty_long_dist_type arg)
  {
    msg_.penalty_long_dist = std::move(arg);
    return Init_FieldDimensions_center_radius(msg_);
  }

private:
  ::rj_msgs::msg::FieldDimensions msg_;
};

class Init_FieldDimensions_penalty_short_dist
{
public:
  explicit Init_FieldDimensions_penalty_short_dist(::rj_msgs::msg::FieldDimensions & msg)
  : msg_(msg)
  {}
  Init_FieldDimensions_penalty_long_dist penalty_short_dist(::rj_msgs::msg::FieldDimensions::_penalty_short_dist_type arg)
  {
    msg_.penalty_short_dist = std::move(arg);
    return Init_FieldDimensions_penalty_long_dist(msg_);
  }

private:
  ::rj_msgs::msg::FieldDimensions msg_;
};

class Init_FieldDimensions_goal_height
{
public:
  explicit Init_FieldDimensions_goal_height(::rj_msgs::msg::FieldDimensions & msg)
  : msg_(msg)
  {}
  Init_FieldDimensions_penalty_short_dist goal_height(::rj_msgs::msg::FieldDimensions::_goal_height_type arg)
  {
    msg_.goal_height = std::move(arg);
    return Init_FieldDimensions_penalty_short_dist(msg_);
  }

private:
  ::rj_msgs::msg::FieldDimensions msg_;
};

class Init_FieldDimensions_goal_depth
{
public:
  explicit Init_FieldDimensions_goal_depth(::rj_msgs::msg::FieldDimensions & msg)
  : msg_(msg)
  {}
  Init_FieldDimensions_goal_height goal_depth(::rj_msgs::msg::FieldDimensions::_goal_depth_type arg)
  {
    msg_.goal_depth = std::move(arg);
    return Init_FieldDimensions_goal_height(msg_);
  }

private:
  ::rj_msgs::msg::FieldDimensions msg_;
};

class Init_FieldDimensions_goal_width
{
public:
  explicit Init_FieldDimensions_goal_width(::rj_msgs::msg::FieldDimensions & msg)
  : msg_(msg)
  {}
  Init_FieldDimensions_goal_depth goal_width(::rj_msgs::msg::FieldDimensions::_goal_width_type arg)
  {
    msg_.goal_width = std::move(arg);
    return Init_FieldDimensions_goal_depth(msg_);
  }

private:
  ::rj_msgs::msg::FieldDimensions msg_;
};

class Init_FieldDimensions_line_width
{
public:
  explicit Init_FieldDimensions_line_width(::rj_msgs::msg::FieldDimensions & msg)
  : msg_(msg)
  {}
  Init_FieldDimensions_goal_width line_width(::rj_msgs::msg::FieldDimensions::_line_width_type arg)
  {
    msg_.line_width = std::move(arg);
    return Init_FieldDimensions_goal_width(msg_);
  }

private:
  ::rj_msgs::msg::FieldDimensions msg_;
};

class Init_FieldDimensions_border
{
public:
  explicit Init_FieldDimensions_border(::rj_msgs::msg::FieldDimensions & msg)
  : msg_(msg)
  {}
  Init_FieldDimensions_line_width border(::rj_msgs::msg::FieldDimensions::_border_type arg)
  {
    msg_.border = std::move(arg);
    return Init_FieldDimensions_line_width(msg_);
  }

private:
  ::rj_msgs::msg::FieldDimensions msg_;
};

class Init_FieldDimensions_width
{
public:
  explicit Init_FieldDimensions_width(::rj_msgs::msg::FieldDimensions & msg)
  : msg_(msg)
  {}
  Init_FieldDimensions_border width(::rj_msgs::msg::FieldDimensions::_width_type arg)
  {
    msg_.width = std::move(arg);
    return Init_FieldDimensions_border(msg_);
  }

private:
  ::rj_msgs::msg::FieldDimensions msg_;
};

class Init_FieldDimensions_length
{
public:
  Init_FieldDimensions_length()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_FieldDimensions_width length(::rj_msgs::msg::FieldDimensions::_length_type arg)
  {
    msg_.length = std::move(arg);
    return Init_FieldDimensions_width(msg_);
  }

private:
  ::rj_msgs::msg::FieldDimensions msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::rj_msgs::msg::FieldDimensions>()
{
  return rj_msgs::msg::builder::Init_FieldDimensions_length();
}

}  // namespace rj_msgs

#endif  // RJ_MSGS__MSG__DETAIL__FIELD_DIMENSIONS__BUILDER_HPP_
