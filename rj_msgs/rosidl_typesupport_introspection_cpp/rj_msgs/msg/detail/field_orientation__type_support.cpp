// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from rj_msgs:msg/FieldOrientation.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "rj_msgs/msg/detail/field_orientation__struct.hpp"
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

void FieldOrientation_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) rj_msgs::msg::FieldOrientation(_init);
}

void FieldOrientation_fini_function(void * message_memory)
{
  auto typed_message = static_cast<rj_msgs::msg::FieldOrientation *>(message_memory);
  typed_message->~FieldOrientation();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember FieldOrientation_message_member_array[1] = {
  {
    "defend_plus_x",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rj_msgs::msg::FieldOrientation, defend_plus_x),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers FieldOrientation_message_members = {
  "rj_msgs::msg",  // message namespace
  "FieldOrientation",  // message name
  1,  // number of fields
  sizeof(rj_msgs::msg::FieldOrientation),
  FieldOrientation_message_member_array,  // message members
  FieldOrientation_init_function,  // function to initialize message memory (memory has to be allocated)
  FieldOrientation_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t FieldOrientation_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &FieldOrientation_message_members,
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
get_message_type_support_handle<rj_msgs::msg::FieldOrientation>()
{
  return &::rj_msgs::msg::rosidl_typesupport_introspection_cpp::FieldOrientation_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, rj_msgs, msg, FieldOrientation)() {
  return &::rj_msgs::msg::rosidl_typesupport_introspection_cpp::FieldOrientation_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
