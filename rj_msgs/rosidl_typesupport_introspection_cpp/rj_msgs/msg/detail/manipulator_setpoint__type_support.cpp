// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from rj_msgs:msg/ManipulatorSetpoint.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "rj_msgs/msg/detail/manipulator_setpoint__struct.hpp"
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

void ManipulatorSetpoint_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) rj_msgs::msg::ManipulatorSetpoint(_init);
}

void ManipulatorSetpoint_fini_function(void * message_memory)
{
  auto typed_message = static_cast<rj_msgs::msg::ManipulatorSetpoint *>(message_memory);
  typed_message->~ManipulatorSetpoint();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember ManipulatorSetpoint_message_member_array[4] = {
  {
    "shoot_mode",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rj_msgs::msg::ManipulatorSetpoint, shoot_mode),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "trigger_mode",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rj_msgs::msg::ManipulatorSetpoint, trigger_mode),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "kick_strength",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rj_msgs::msg::ManipulatorSetpoint, kick_strength),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "dribbler_speed",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rj_msgs::msg::ManipulatorSetpoint, dribbler_speed),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers ManipulatorSetpoint_message_members = {
  "rj_msgs::msg",  // message namespace
  "ManipulatorSetpoint",  // message name
  4,  // number of fields
  sizeof(rj_msgs::msg::ManipulatorSetpoint),
  ManipulatorSetpoint_message_member_array,  // message members
  ManipulatorSetpoint_init_function,  // function to initialize message memory (memory has to be allocated)
  ManipulatorSetpoint_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t ManipulatorSetpoint_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &ManipulatorSetpoint_message_members,
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
get_message_type_support_handle<rj_msgs::msg::ManipulatorSetpoint>()
{
  return &::rj_msgs::msg::rosidl_typesupport_introspection_cpp::ManipulatorSetpoint_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, rj_msgs, msg, ManipulatorSetpoint)() {
  return &::rj_msgs::msg::rosidl_typesupport_introspection_cpp::ManipulatorSetpoint_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
