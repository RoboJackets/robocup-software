// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from rj_msgs:msg/MotionSetpoint.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "rj_msgs/msg/detail/motion_setpoint__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace rj_msgs
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void MotionSetpoint_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) rj_msgs::msg::MotionSetpoint(_init);
}

void MotionSetpoint_fini_function(void * message_memory)
{
  auto typed_message = static_cast<rj_msgs::msg::MotionSetpoint *>(message_memory);
  typed_message->~MotionSetpoint();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember MotionSetpoint_message_member_array[3] = {
  {
    "velocity_x_mps",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rj_msgs::msg::MotionSetpoint, velocity_x_mps),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "velocity_y_mps",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rj_msgs::msg::MotionSetpoint, velocity_y_mps),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "velocity_z_radps",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rj_msgs::msg::MotionSetpoint, velocity_z_radps),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers MotionSetpoint_message_members = {
  "rj_msgs::msg",  // message namespace
  "MotionSetpoint",  // message name
  3,  // number of fields
  sizeof(rj_msgs::msg::MotionSetpoint),
  MotionSetpoint_message_member_array,  // message members
  MotionSetpoint_init_function,  // function to initialize message memory (memory has to be allocated)
  MotionSetpoint_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t MotionSetpoint_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &MotionSetpoint_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace rj_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<rj_msgs::msg::MotionSetpoint>()
{
  return &::rj_msgs::msg::rosidl_typesupport_introspection_cpp::MotionSetpoint_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, rj_msgs, msg, MotionSetpoint)() {
  return &::rj_msgs::msg::rosidl_typesupport_introspection_cpp::MotionSetpoint_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
