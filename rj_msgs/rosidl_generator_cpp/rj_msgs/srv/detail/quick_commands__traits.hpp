// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from rj_msgs:srv/QuickCommands.idl
// generated code does not contain a copyright notice

#ifndef RJ_MSGS__SRV__DETAIL__QUICK_COMMANDS__TRAITS_HPP_
#define RJ_MSGS__SRV__DETAIL__QUICK_COMMANDS__TRAITS_HPP_

#include "rj_msgs/srv/detail/quick_commands__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<rj_msgs::srv::QuickCommands_Request>()
{
  return "rj_msgs::srv::QuickCommands_Request";
}

template<>
inline const char * name<rj_msgs::srv::QuickCommands_Request>()
{
  return "rj_msgs/srv/QuickCommands_Request";
}

template<>
struct has_fixed_size<rj_msgs::srv::QuickCommands_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<rj_msgs::srv::QuickCommands_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<rj_msgs::srv::QuickCommands_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<rj_msgs::srv::QuickCommands_Response>()
{
  return "rj_msgs::srv::QuickCommands_Response";
}

template<>
inline const char * name<rj_msgs::srv::QuickCommands_Response>()
{
  return "rj_msgs/srv/QuickCommands_Response";
}

template<>
struct has_fixed_size<rj_msgs::srv::QuickCommands_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<rj_msgs::srv::QuickCommands_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<rj_msgs::srv::QuickCommands_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<rj_msgs::srv::QuickCommands>()
{
  return "rj_msgs::srv::QuickCommands";
}

template<>
inline const char * name<rj_msgs::srv::QuickCommands>()
{
  return "rj_msgs/srv/QuickCommands";
}

template<>
struct has_fixed_size<rj_msgs::srv::QuickCommands>
  : std::integral_constant<
    bool,
    has_fixed_size<rj_msgs::srv::QuickCommands_Request>::value &&
    has_fixed_size<rj_msgs::srv::QuickCommands_Response>::value
  >
{
};

template<>
struct has_bounded_size<rj_msgs::srv::QuickCommands>
  : std::integral_constant<
    bool,
    has_bounded_size<rj_msgs::srv::QuickCommands_Request>::value &&
    has_bounded_size<rj_msgs::srv::QuickCommands_Response>::value
  >
{
};

template<>
struct is_service<rj_msgs::srv::QuickCommands>
  : std::true_type
{
};

template<>
struct is_service_request<rj_msgs::srv::QuickCommands_Request>
  : std::true_type
{
};

template<>
struct is_service_response<rj_msgs::srv::QuickCommands_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // RJ_MSGS__SRV__DETAIL__QUICK_COMMANDS__TRAITS_HPP_
