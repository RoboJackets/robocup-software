// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rj_msgs:msg/DetectionFrame.idl
// generated code does not contain a copyright notice

#ifndef RJ_MSGS__MSG__DETAIL__DETECTION_FRAME__BUILDER_HPP_
#define RJ_MSGS__MSG__DETAIL__DETECTION_FRAME__BUILDER_HPP_

#include "rj_msgs/msg/detail/detection_frame__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace rj_msgs
{

namespace msg
{

namespace builder
{

class Init_DetectionFrame_robots_blue
{
public:
  explicit Init_DetectionFrame_robots_blue(::rj_msgs::msg::DetectionFrame & msg)
  : msg_(msg)
  {}
  ::rj_msgs::msg::DetectionFrame robots_blue(::rj_msgs::msg::DetectionFrame::_robots_blue_type arg)
  {
    msg_.robots_blue = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rj_msgs::msg::DetectionFrame msg_;
};

class Init_DetectionFrame_robots_yellow
{
public:
  explicit Init_DetectionFrame_robots_yellow(::rj_msgs::msg::DetectionFrame & msg)
  : msg_(msg)
  {}
  Init_DetectionFrame_robots_blue robots_yellow(::rj_msgs::msg::DetectionFrame::_robots_yellow_type arg)
  {
    msg_.robots_yellow = std::move(arg);
    return Init_DetectionFrame_robots_blue(msg_);
  }

private:
  ::rj_msgs::msg::DetectionFrame msg_;
};

class Init_DetectionFrame_balls
{
public:
  explicit Init_DetectionFrame_balls(::rj_msgs::msg::DetectionFrame & msg)
  : msg_(msg)
  {}
  Init_DetectionFrame_robots_yellow balls(::rj_msgs::msg::DetectionFrame::_balls_type arg)
  {
    msg_.balls = std::move(arg);
    return Init_DetectionFrame_robots_yellow(msg_);
  }

private:
  ::rj_msgs::msg::DetectionFrame msg_;
};

class Init_DetectionFrame_camera_id
{
public:
  explicit Init_DetectionFrame_camera_id(::rj_msgs::msg::DetectionFrame & msg)
  : msg_(msg)
  {}
  Init_DetectionFrame_balls camera_id(::rj_msgs::msg::DetectionFrame::_camera_id_type arg)
  {
    msg_.camera_id = std::move(arg);
    return Init_DetectionFrame_balls(msg_);
  }

private:
  ::rj_msgs::msg::DetectionFrame msg_;
};

class Init_DetectionFrame_t_received
{
public:
  explicit Init_DetectionFrame_t_received(::rj_msgs::msg::DetectionFrame & msg)
  : msg_(msg)
  {}
  Init_DetectionFrame_camera_id t_received(::rj_msgs::msg::DetectionFrame::_t_received_type arg)
  {
    msg_.t_received = std::move(arg);
    return Init_DetectionFrame_camera_id(msg_);
  }

private:
  ::rj_msgs::msg::DetectionFrame msg_;
};

class Init_DetectionFrame_t_sent
{
public:
  explicit Init_DetectionFrame_t_sent(::rj_msgs::msg::DetectionFrame & msg)
  : msg_(msg)
  {}
  Init_DetectionFrame_t_received t_sent(::rj_msgs::msg::DetectionFrame::_t_sent_type arg)
  {
    msg_.t_sent = std::move(arg);
    return Init_DetectionFrame_t_received(msg_);
  }

private:
  ::rj_msgs::msg::DetectionFrame msg_;
};

class Init_DetectionFrame_t_capture
{
public:
  explicit Init_DetectionFrame_t_capture(::rj_msgs::msg::DetectionFrame & msg)
  : msg_(msg)
  {}
  Init_DetectionFrame_t_sent t_capture(::rj_msgs::msg::DetectionFrame::_t_capture_type arg)
  {
    msg_.t_capture = std::move(arg);
    return Init_DetectionFrame_t_sent(msg_);
  }

private:
  ::rj_msgs::msg::DetectionFrame msg_;
};

class Init_DetectionFrame_frame_number
{
public:
  Init_DetectionFrame_frame_number()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DetectionFrame_t_capture frame_number(::rj_msgs::msg::DetectionFrame::_frame_number_type arg)
  {
    msg_.frame_number = std::move(arg);
    return Init_DetectionFrame_t_capture(msg_);
  }

private:
  ::rj_msgs::msg::DetectionFrame msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::rj_msgs::msg::DetectionFrame>()
{
  return rj_msgs::msg::builder::Init_DetectionFrame_frame_number();
}

}  // namespace rj_msgs

#endif  // RJ_MSGS__MSG__DETAIL__DETECTION_FRAME__BUILDER_HPP_
