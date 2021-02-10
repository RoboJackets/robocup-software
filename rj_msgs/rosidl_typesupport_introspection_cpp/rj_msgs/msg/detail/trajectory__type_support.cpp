// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from rj_msgs:msg/Trajectory.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "rj_msgs/msg/detail/trajectory__struct.hpp"
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

void Trajectory_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) rj_msgs::msg::Trajectory(_init);
}

void Trajectory_fini_function(void * message_memory)
{
  auto typed_message = static_cast<rj_msgs::msg::Trajectory *>(message_memory);
  typed_message->~Trajectory();
}

size_t size_function__Trajectory__instants(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<rj_msgs::msg::RobotInstant> *>(untyped_member);
  return member->size();
}

const void * get_const_function__Trajectory__instants(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<rj_msgs::msg::RobotInstant> *>(untyped_member);
  return &member[index];
}

void * get_function__Trajectory__instants(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<rj_msgs::msg::RobotInstant> *>(untyped_member);
  return &member[index];
}

void resize_function__Trajectory__instants(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<rj_msgs::msg::RobotInstant> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember Trajectory_message_member_array[2] = {
  {
    "stamp",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<builtin_interfaces::msg::Time>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rj_msgs::msg::Trajectory, stamp),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "instants",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<rj_msgs::msg::RobotInstant>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rj_msgs::msg::Trajectory, instants),  // bytes offset in struct
    nullptr,  // default value
    size_function__Trajectory__instants,  // size() function pointer
    get_const_function__Trajectory__instants,  // get_const(index) function pointer
    get_function__Trajectory__instants,  // get(index) function pointer
    resize_function__Trajectory__instants  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers Trajectory_message_members = {
  "rj_msgs::msg",  // message namespace
  "Trajectory",  // message name
  2,  // number of fields
  sizeof(rj_msgs::msg::Trajectory),
  Trajectory_message_member_array,  // message members
  Trajectory_init_function,  // function to initialize message memory (memory has to be allocated)
  Trajectory_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t Trajectory_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &Trajectory_message_members,
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
get_message_type_support_handle<rj_msgs::msg::Trajectory>()
{
  return &::rj_msgs::msg::rosidl_typesupport_introspection_cpp::Trajectory_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, rj_msgs, msg, Trajectory)() {
  return &::rj_msgs::msg::rosidl_typesupport_introspection_cpp::Trajectory_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
