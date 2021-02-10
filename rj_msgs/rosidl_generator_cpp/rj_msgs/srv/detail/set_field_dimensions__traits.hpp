// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from rj_msgs:srv/SetFieldDimensions.idl
// generated code does not contain a copyright notice

#ifndef RJ_MSGS__SRV__DETAIL__SET_FIELD_DIMENSIONS__TRAITS_HPP_
#define RJ_MSGS__SRV__DETAIL__SET_FIELD_DIMENSIONS__TRAITS_HPP_

#include "rj_msgs/srv/detail/set_field_dimensions__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'field_dimensions'
#include "rj_msgs/msg/detail/field_dimensions__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<rj_msgs::srv::SetFieldDimensions_Request>()
{
  return "rj_msgs::srv::SetFieldDimensions_Request";
}

template<>
inline const char * name<rj_msgs::srv::SetFieldDimensions_Request>()
{
  return "rj_msgs/srv/SetFieldDimensions_Request";
}

template<>
struct has_fixed_size<rj_msgs::srv::SetFieldDimensions_Request>
  : std::integral_constant<bool, has_fixed_size<rj_msgs::msg::FieldDimensions>::value> {};

template<>
struct has_bounded_size<rj_msgs::srv::SetFieldDimensions_Request>
  : std::integral_constant<bool, has_bounded_size<rj_msgs::msg::FieldDimensions>::value> {};

template<>
struct is_message<rj_msgs::srv::SetFieldDimensions_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<rj_msgs::srv::SetFieldDimensions_Response>()
{
  return "rj_msgs::srv::SetFieldDimensions_Response";
}

template<>
inline const char * name<rj_msgs::srv::SetFieldDimensions_Response>()
{
  return "rj_msgs/srv/SetFieldDimensions_Response";
}

template<>
struct has_fixed_size<rj_msgs::srv::SetFieldDimensions_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<rj_msgs::srv::SetFieldDimensions_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<rj_msgs::srv::SetFieldDimensions_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<rj_msgs::srv::SetFieldDimensions>()
{
  return "rj_msgs::srv::SetFieldDimensions";
}

template<>
inline const char * name<rj_msgs::srv::SetFieldDimensions>()
{
  return "rj_msgs/srv/SetFieldDimensions";
}

template<>
struct has_fixed_size<rj_msgs::srv::SetFieldDimensions>
  : std::integral_constant<
    bool,
    has_fixed_size<rj_msgs::srv::SetFieldDimensions_Request>::value &&
    has_fixed_size<rj_msgs::srv::SetFieldDimensions_Response>::value
  >
{
};

template<>
struct has_bounded_size<rj_msgs::srv::SetFieldDimensions>
  : std::integral_constant<
    bool,
    has_bounded_size<rj_msgs::srv::SetFieldDimensions_Request>::value &&
    has_bounded_size<rj_msgs::srv::SetFieldDimensions_Response>::value
  >
{
};

template<>
struct is_service<rj_msgs::srv::SetFieldDimensions>
  : std::true_type
{
};

template<>
struct is_service_request<rj_msgs::srv::SetFieldDimensions_Request>
  : std::true_type
{
};

template<>
struct is_service_response<rj_msgs::srv::SetFieldDimensions_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // RJ_MSGS__SRV__DETAIL__SET_FIELD_DIMENSIONS__TRAITS_HPP_
