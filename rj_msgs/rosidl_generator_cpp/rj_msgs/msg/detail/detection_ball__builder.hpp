// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rj_msgs:msg/DetectionBall.idl
// generated code does not contain a copyright notice

#ifndef RJ_MSGS__MSG__DETAIL__DETECTION_BALL__BUILDER_HPP_
#define RJ_MSGS__MSG__DETAIL__DETECTION_BALL__BUILDER_HPP_

#include "rj_msgs/msg/detail/detection_ball__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace rj_msgs
{

namespace msg
{

namespace builder
{

class Init_DetectionBall_pixel_y
{
public:
  explicit Init_DetectionBall_pixel_y(::rj_msgs::msg::DetectionBall & msg)
  : msg_(msg)
  {}
  ::rj_msgs::msg::DetectionBall pixel_y(::rj_msgs::msg::DetectionBall::_pixel_y_type arg)
  {
    msg_.pixel_y = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rj_msgs::msg::DetectionBall msg_;
};

class Init_DetectionBall_pixel_x
{
public:
  explicit Init_DetectionBall_pixel_x(::rj_msgs::msg::DetectionBall & msg)
  : msg_(msg)
  {}
  Init_DetectionBall_pixel_y pixel_x(::rj_msgs::msg::DetectionBall::_pixel_x_type arg)
  {
    msg_.pixel_x = std::move(arg);
    return Init_DetectionBall_pixel_y(msg_);
  }

private:
  ::rj_msgs::msg::DetectionBall msg_;
};

class Init_DetectionBall_z
{
public:
  explicit Init_DetectionBall_z(::rj_msgs::msg::DetectionBall & msg)
  : msg_(msg)
  {}
  Init_DetectionBall_pixel_x z(::rj_msgs::msg::DetectionBall::_z_type arg)
  {
    msg_.z = std::move(arg);
    return Init_DetectionBall_pixel_x(msg_);
  }

private:
  ::rj_msgs::msg::DetectionBall msg_;
};

class Init_DetectionBall_y
{
public:
  explicit Init_DetectionBall_y(::rj_msgs::msg::DetectionBall & msg)
  : msg_(msg)
  {}
  Init_DetectionBall_z y(::rj_msgs::msg::DetectionBall::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_DetectionBall_z(msg_);
  }

private:
  ::rj_msgs::msg::DetectionBall msg_;
};

class Init_DetectionBall_x
{
public:
  explicit Init_DetectionBall_x(::rj_msgs::msg::DetectionBall & msg)
  : msg_(msg)
  {}
  Init_DetectionBall_y x(::rj_msgs::msg::DetectionBall::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_DetectionBall_y(msg_);
  }

private:
  ::rj_msgs::msg::DetectionBall msg_;
};

class Init_DetectionBall_area
{
public:
  explicit Init_DetectionBall_area(::rj_msgs::msg::DetectionBall & msg)
  : msg_(msg)
  {}
  Init_DetectionBall_x area(::rj_msgs::msg::DetectionBall::_area_type arg)
  {
    msg_.area = std::move(arg);
    return Init_DetectionBall_x(msg_);
  }

private:
  ::rj_msgs::msg::DetectionBall msg_;
};

class Init_DetectionBall_confidence
{
public:
  Init_DetectionBall_confidence()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DetectionBall_area confidence(::rj_msgs::msg::DetectionBall::_confidence_type arg)
  {
    msg_.confidence = std::move(arg);
    return Init_DetectionBall_area(msg_);
  }

private:
  ::rj_msgs::msg::DetectionBall msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::rj_msgs::msg::DetectionBall>()
{
  return rj_msgs::msg::builder::Init_DetectionBall_confidence();
}

}  // namespace rj_msgs

#endif  // RJ_MSGS__MSG__DETAIL__DETECTION_BALL__BUILDER_HPP_
