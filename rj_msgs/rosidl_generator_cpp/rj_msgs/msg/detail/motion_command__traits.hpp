// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from rj_msgs:msg/MotionCommand.idl
// generated code does not contain a copyright notice

#ifndef RJ_MSGS__MSG__DETAIL__MOTION_COMMAND__TRAITS_HPP_
#define RJ_MSGS__MSG__DETAIL__MOTION_COMMAND__TRAITS_HPP_

#include "rj_msgs/msg/detail/motion_command__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'empty_command'
#include "rj_msgs/msg/detail/empty_motion_command__traits.hpp"
// Member 'path_target_command'
#include "rj_msgs/msg/detail/path_target_motion_command__traits.hpp"
// Member 'world_vel_command'
#include "rj_msgs/msg/detail/world_vel_motion_command__traits.hpp"
// Member 'pivot_command'
#include "rj_msgs/msg/detail/pivot_motion_command__traits.hpp"
// Member 'settle_command'
#include "rj_msgs/msg/detail/settle_motion_command__traits.hpp"
// Member 'collect_command'
#include "rj_msgs/msg/detail/collect_motion_command__traits.hpp"
// Member 'line_kick_command'
#include "rj_msgs/msg/detail/line_kick_motion_command__traits.hpp"
// Member 'intercept_command'
#include "rj_msgs/msg/detail/intercept_motion_command__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<rj_msgs::msg::MotionCommand>()
{
  return "rj_msgs::msg::MotionCommand";
}

template<>
inline const char * name<rj_msgs::msg::MotionCommand>()
{
  return "rj_msgs/msg/MotionCommand";
}

template<>
struct has_fixed_size<rj_msgs::msg::MotionCommand>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<rj_msgs::msg::MotionCommand>
  : std::integral_constant<bool, has_bounded_size<rj_msgs::msg::CollectMotionCommand>::value && has_bounded_size<rj_msgs::msg::EmptyMotionCommand>::value && has_bounded_size<rj_msgs::msg::InterceptMotionCommand>::value && has_bounded_size<rj_msgs::msg::LineKickMotionCommand>::value && has_bounded_size<rj_msgs::msg::PathTargetMotionCommand>::value && has_bounded_size<rj_msgs::msg::PivotMotionCommand>::value && has_bounded_size<rj_msgs::msg::SettleMotionCommand>::value && has_bounded_size<rj_msgs::msg::WorldVelMotionCommand>::value> {};

template<>
struct is_message<rj_msgs::msg::MotionCommand>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // RJ_MSGS__MSG__DETAIL__MOTION_COMMAND__TRAITS_HPP_
