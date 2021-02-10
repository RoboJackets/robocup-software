// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from rj_msgs:srv/QuickRestart.idl
// generated code does not contain a copyright notice

#ifndef RJ_MSGS__SRV__DETAIL__QUICK_RESTART__TRAITS_HPP_
#define RJ_MSGS__SRV__DETAIL__QUICK_RESTART__TRAITS_HPP_

#include "rj_msgs/srv/detail/quick_restart__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<rj_msgs::srv::QuickRestart_Request>()
{
  return "rj_msgs::srv::QuickRestart_Request";
}

template<>
inline const char * name<rj_msgs::srv::QuickRestart_Request>()
{
  return "rj_msgs/srv/QuickRestart_Request";
}

template<>
struct has_fixed_size<rj_msgs::srv::QuickRestart_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<rj_msgs::srv::QuickRestart_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<rj_msgs::srv::QuickRestart_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<rj_msgs::srv::QuickRestart_Response>()
{
  return "rj_msgs::srv::QuickRestart_Response";
}

template<>
inline const char * name<rj_msgs::srv::QuickRestart_Response>()
{
  return "rj_msgs/srv/QuickRestart_Response";
}

template<>
struct has_fixed_size<rj_msgs::srv::QuickRestart_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<rj_msgs::srv::QuickRestart_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<rj_msgs::srv::QuickRestart_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<rj_msgs::srv::QuickRestart>()
{
  return "rj_msgs::srv::QuickRestart";
}

template<>
inline const char * name<rj_msgs::srv::QuickRestart>()
{
  return "rj_msgs/srv/QuickRestart";
}

template<>
struct has_fixed_size<rj_msgs::srv::QuickRestart>
  : std::integral_constant<
    bool,
    has_fixed_size<rj_msgs::srv::QuickRestart_Request>::value &&
    has_fixed_size<rj_msgs::srv::QuickRestart_Response>::value
  >
{
};

template<>
struct has_bounded_size<rj_msgs::srv::QuickRestart>
  : std::integral_constant<
    bool,
    has_bounded_size<rj_msgs::srv::QuickRestart_Request>::value &&
    has_bounded_size<rj_msgs::srv::QuickRestart_Response>::value
  >
{
};

template<>
struct is_service<rj_msgs::srv::QuickRestart>
  : std::true_type
{
};

template<>
struct is_service_request<rj_msgs::srv::QuickRestart_Request>
  : std::true_type
{
};

template<>
struct is_service_response<rj_msgs::srv::QuickRestart_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // RJ_MSGS__SRV__DETAIL__QUICK_RESTART__TRAITS_HPP_
