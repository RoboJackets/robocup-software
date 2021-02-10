// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from rj_msgs:msg/WorldState.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "rj_msgs/msg/detail/world_state__struct.hpp"
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

void WorldState_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) rj_msgs::msg::WorldState(_init);
}

void WorldState_fini_function(void * message_memory)
{
  auto typed_message = static_cast<rj_msgs::msg::WorldState *>(message_memory);
  typed_message->~WorldState();
}

size_t size_function__WorldState__their_robots(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<rj_msgs::msg::RobotState> *>(untyped_member);
  return member->size();
}

const void * get_const_function__WorldState__their_robots(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<rj_msgs::msg::RobotState> *>(untyped_member);
  return &member[index];
}

void * get_function__WorldState__their_robots(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<rj_msgs::msg::RobotState> *>(untyped_member);
  return &member[index];
}

void resize_function__WorldState__their_robots(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<rj_msgs::msg::RobotState> *>(untyped_member);
  member->resize(size);
}

size_t size_function__WorldState__our_robots(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<rj_msgs::msg::RobotState> *>(untyped_member);
  return member->size();
}

const void * get_const_function__WorldState__our_robots(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<rj_msgs::msg::RobotState> *>(untyped_member);
  return &member[index];
}

void * get_function__WorldState__our_robots(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<rj_msgs::msg::RobotState> *>(untyped_member);
  return &member[index];
}

void resize_function__WorldState__our_robots(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<rj_msgs::msg::RobotState> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember WorldState_message_member_array[4] = {
  {
    "last_update_time",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<builtin_interfaces::msg::Time>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rj_msgs::msg::WorldState, last_update_time),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "their_robots",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<rj_msgs::msg::RobotState>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rj_msgs::msg::WorldState, their_robots),  // bytes offset in struct
    nullptr,  // default value
    size_function__WorldState__their_robots,  // size() function pointer
    get_const_function__WorldState__their_robots,  // get_const(index) function pointer
    get_function__WorldState__their_robots,  // get(index) function pointer
    resize_function__WorldState__their_robots  // resize(index) function pointer
  },
  {
    "our_robots",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<rj_msgs::msg::RobotState>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rj_msgs::msg::WorldState, our_robots),  // bytes offset in struct
    nullptr,  // default value
    size_function__WorldState__our_robots,  // size() function pointer
    get_const_function__WorldState__our_robots,  // get_const(index) function pointer
    get_function__WorldState__our_robots,  // get(index) function pointer
    resize_function__WorldState__our_robots  // resize(index) function pointer
  },
  {
    "ball",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<rj_msgs::msg::BallState>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rj_msgs::msg::WorldState, ball),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers WorldState_message_members = {
  "rj_msgs::msg",  // message namespace
  "WorldState",  // message name
  4,  // number of fields
  sizeof(rj_msgs::msg::WorldState),
  WorldState_message_member_array,  // message members
  WorldState_init_function,  // function to initialize message memory (memory has to be allocated)
  WorldState_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t WorldState_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &WorldState_message_members,
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
get_message_type_support_handle<rj_msgs::msg::WorldState>()
{
  return &::rj_msgs::msg::rosidl_typesupport_introspection_cpp::WorldState_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, rj_msgs, msg, WorldState)() {
  return &::rj_msgs::msg::rosidl_typesupport_introspection_cpp::WorldState_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
