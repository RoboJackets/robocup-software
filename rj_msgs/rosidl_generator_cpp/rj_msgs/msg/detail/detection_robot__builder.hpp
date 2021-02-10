// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rj_msgs:msg/DetectionRobot.idl
// generated code does not contain a copyright notice

#ifndef RJ_MSGS__MSG__DETAIL__DETECTION_ROBOT__BUILDER_HPP_
#define RJ_MSGS__MSG__DETAIL__DETECTION_ROBOT__BUILDER_HPP_

#include "rj_msgs/msg/detail/detection_robot__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace rj_msgs
{

namespace msg
{

namespace builder
{

class Init_DetectionRobot_height
{
public:
  explicit Init_DetectionRobot_height(::rj_msgs::msg::DetectionRobot & msg)
  : msg_(msg)
  {}
  ::rj_msgs::msg::DetectionRobot height(::rj_msgs::msg::DetectionRobot::_height_type arg)
  {
    msg_.height = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rj_msgs::msg::DetectionRobot msg_;
};

class Init_DetectionRobot_pixel_y
{
public:
  explicit Init_DetectionRobot_pixel_y(::rj_msgs::msg::DetectionRobot & msg)
  : msg_(msg)
  {}
  Init_DetectionRobot_height pixel_y(::rj_msgs::msg::DetectionRobot::_pixel_y_type arg)
  {
    msg_.pixel_y = std::move(arg);
    return Init_DetectionRobot_height(msg_);
  }

private:
  ::rj_msgs::msg::DetectionRobot msg_;
};

class Init_DetectionRobot_pixel_x
{
public:
  explicit Init_DetectionRobot_pixel_x(::rj_msgs::msg::DetectionRobot & msg)
  : msg_(msg)
  {}
  Init_DetectionRobot_pixel_y pixel_x(::rj_msgs::msg::DetectionRobot::_pixel_x_type arg)
  {
    msg_.pixel_x = std::move(arg);
    return Init_DetectionRobot_pixel_y(msg_);
  }

private:
  ::rj_msgs::msg::DetectionRobot msg_;
};

class Init_DetectionRobot_orientation
{
public:
  explicit Init_DetectionRobot_orientation(::rj_msgs::msg::DetectionRobot & msg)
  : msg_(msg)
  {}
  Init_DetectionRobot_pixel_x orientation(::rj_msgs::msg::DetectionRobot::_orientation_type arg)
  {
    msg_.orientation = std::move(arg);
    return Init_DetectionRobot_pixel_x(msg_);
  }

private:
  ::rj_msgs::msg::DetectionRobot msg_;
};

class Init_DetectionRobot_y
{
public:
  explicit Init_DetectionRobot_y(::rj_msgs::msg::DetectionRobot & msg)
  : msg_(msg)
  {}
  Init_DetectionRobot_orientation y(::rj_msgs::msg::DetectionRobot::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_DetectionRobot_orientation(msg_);
  }

private:
  ::rj_msgs::msg::DetectionRobot msg_;
};

class Init_DetectionRobot_x
{
public:
  explicit Init_DetectionRobot_x(::rj_msgs::msg::DetectionRobot & msg)
  : msg_(msg)
  {}
  Init_DetectionRobot_y x(::rj_msgs::msg::DetectionRobot::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_DetectionRobot_y(msg_);
  }

private:
  ::rj_msgs::msg::DetectionRobot msg_;
};

class Init_DetectionRobot_robot_id
{
public:
  explicit Init_DetectionRobot_robot_id(::rj_msgs::msg::DetectionRobot & msg)
  : msg_(msg)
  {}
  Init_DetectionRobot_x robot_id(::rj_msgs::msg::DetectionRobot::_robot_id_type arg)
  {
    msg_.robot_id = std::move(arg);
    return Init_DetectionRobot_x(msg_);
  }

private:
  ::rj_msgs::msg::DetectionRobot msg_;
};

class Init_DetectionRobot_confidence
{
public:
  Init_DetectionRobot_confidence()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DetectionRobot_robot_id confidence(::rj_msgs::msg::DetectionRobot::_confidence_type arg)
  {
    msg_.confidence = std::move(arg);
    return Init_DetectionRobot_robot_id(msg_);
  }

private:
  ::rj_msgs::msg::DetectionRobot msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::rj_msgs::msg::DetectionRobot>()
{
  return rj_msgs::msg::builder::Init_DetectionRobot_confidence();
}

}  // namespace rj_msgs

#endif  // RJ_MSGS__MSG__DETAIL__DETECTION_ROBOT__BUILDER_HPP_
