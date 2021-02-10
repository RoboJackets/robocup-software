// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from rj_msgs:msg/MotionCommand.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "rj_msgs/msg/detail/motion_command__struct.hpp"
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

void MotionCommand_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) rj_msgs::msg::MotionCommand(_init);
}

void MotionCommand_fini_function(void * message_memory)
{
  auto typed_message = static_cast<rj_msgs::msg::MotionCommand *>(message_memory);
  typed_message->~MotionCommand();
}

size_t size_function__MotionCommand__empty_command(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<rj_msgs::msg::EmptyMotionCommand> *>(untyped_member);
  return member->size();
}

const void * get_const_function__MotionCommand__empty_command(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<rj_msgs::msg::EmptyMotionCommand> *>(untyped_member);
  return &member[index];
}

void * get_function__MotionCommand__empty_command(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<rj_msgs::msg::EmptyMotionCommand> *>(untyped_member);
  return &member[index];
}

void resize_function__MotionCommand__empty_command(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<rj_msgs::msg::EmptyMotionCommand> *>(untyped_member);
  member->resize(size);
}

size_t size_function__MotionCommand__path_target_command(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<rj_msgs::msg::PathTargetMotionCommand> *>(untyped_member);
  return member->size();
}

const void * get_const_function__MotionCommand__path_target_command(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<rj_msgs::msg::PathTargetMotionCommand> *>(untyped_member);
  return &member[index];
}

void * get_function__MotionCommand__path_target_command(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<rj_msgs::msg::PathTargetMotionCommand> *>(untyped_member);
  return &member[index];
}

void resize_function__MotionCommand__path_target_command(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<rj_msgs::msg::PathTargetMotionCommand> *>(untyped_member);
  member->resize(size);
}

size_t size_function__MotionCommand__world_vel_command(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<rj_msgs::msg::WorldVelMotionCommand> *>(untyped_member);
  return member->size();
}

const void * get_const_function__MotionCommand__world_vel_command(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<rj_msgs::msg::WorldVelMotionCommand> *>(untyped_member);
  return &member[index];
}

void * get_function__MotionCommand__world_vel_command(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<rj_msgs::msg::WorldVelMotionCommand> *>(untyped_member);
  return &member[index];
}

void resize_function__MotionCommand__world_vel_command(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<rj_msgs::msg::WorldVelMotionCommand> *>(untyped_member);
  member->resize(size);
}

size_t size_function__MotionCommand__pivot_command(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<rj_msgs::msg::PivotMotionCommand> *>(untyped_member);
  return member->size();
}

const void * get_const_function__MotionCommand__pivot_command(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<rj_msgs::msg::PivotMotionCommand> *>(untyped_member);
  return &member[index];
}

void * get_function__MotionCommand__pivot_command(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<rj_msgs::msg::PivotMotionCommand> *>(untyped_member);
  return &member[index];
}

void resize_function__MotionCommand__pivot_command(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<rj_msgs::msg::PivotMotionCommand> *>(untyped_member);
  member->resize(size);
}

size_t size_function__MotionCommand__settle_command(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<rj_msgs::msg::SettleMotionCommand> *>(untyped_member);
  return member->size();
}

const void * get_const_function__MotionCommand__settle_command(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<rj_msgs::msg::SettleMotionCommand> *>(untyped_member);
  return &member[index];
}

void * get_function__MotionCommand__settle_command(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<rj_msgs::msg::SettleMotionCommand> *>(untyped_member);
  return &member[index];
}

void resize_function__MotionCommand__settle_command(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<rj_msgs::msg::SettleMotionCommand> *>(untyped_member);
  member->resize(size);
}

size_t size_function__MotionCommand__collect_command(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<rj_msgs::msg::CollectMotionCommand> *>(untyped_member);
  return member->size();
}

const void * get_const_function__MotionCommand__collect_command(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<rj_msgs::msg::CollectMotionCommand> *>(untyped_member);
  return &member[index];
}

void * get_function__MotionCommand__collect_command(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<rj_msgs::msg::CollectMotionCommand> *>(untyped_member);
  return &member[index];
}

void resize_function__MotionCommand__collect_command(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<rj_msgs::msg::CollectMotionCommand> *>(untyped_member);
  member->resize(size);
}

size_t size_function__MotionCommand__line_kick_command(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<rj_msgs::msg::LineKickMotionCommand> *>(untyped_member);
  return member->size();
}

const void * get_const_function__MotionCommand__line_kick_command(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<rj_msgs::msg::LineKickMotionCommand> *>(untyped_member);
  return &member[index];
}

void * get_function__MotionCommand__line_kick_command(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<rj_msgs::msg::LineKickMotionCommand> *>(untyped_member);
  return &member[index];
}

void resize_function__MotionCommand__line_kick_command(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<rj_msgs::msg::LineKickMotionCommand> *>(untyped_member);
  member->resize(size);
}

size_t size_function__MotionCommand__intercept_command(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<rj_msgs::msg::InterceptMotionCommand> *>(untyped_member);
  return member->size();
}

const void * get_const_function__MotionCommand__intercept_command(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<rj_msgs::msg::InterceptMotionCommand> *>(untyped_member);
  return &member[index];
}

void * get_function__MotionCommand__intercept_command(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<rj_msgs::msg::InterceptMotionCommand> *>(untyped_member);
  return &member[index];
}

void resize_function__MotionCommand__intercept_command(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<rj_msgs::msg::InterceptMotionCommand> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember MotionCommand_message_member_array[8] = {
  {
    "empty_command",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<rj_msgs::msg::EmptyMotionCommand>(),  // members of sub message
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(rj_msgs::msg::MotionCommand, empty_command),  // bytes offset in struct
    nullptr,  // default value
    size_function__MotionCommand__empty_command,  // size() function pointer
    get_const_function__MotionCommand__empty_command,  // get_const(index) function pointer
    get_function__MotionCommand__empty_command,  // get(index) function pointer
    resize_function__MotionCommand__empty_command  // resize(index) function pointer
  },
  {
    "path_target_command",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<rj_msgs::msg::PathTargetMotionCommand>(),  // members of sub message
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(rj_msgs::msg::MotionCommand, path_target_command),  // bytes offset in struct
    nullptr,  // default value
    size_function__MotionCommand__path_target_command,  // size() function pointer
    get_const_function__MotionCommand__path_target_command,  // get_const(index) function pointer
    get_function__MotionCommand__path_target_command,  // get(index) function pointer
    resize_function__MotionCommand__path_target_command  // resize(index) function pointer
  },
  {
    "world_vel_command",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<rj_msgs::msg::WorldVelMotionCommand>(),  // members of sub message
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(rj_msgs::msg::MotionCommand, world_vel_command),  // bytes offset in struct
    nullptr,  // default value
    size_function__MotionCommand__world_vel_command,  // size() function pointer
    get_const_function__MotionCommand__world_vel_command,  // get_const(index) function pointer
    get_function__MotionCommand__world_vel_command,  // get(index) function pointer
    resize_function__MotionCommand__world_vel_command  // resize(index) function pointer
  },
  {
    "pivot_command",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<rj_msgs::msg::PivotMotionCommand>(),  // members of sub message
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(rj_msgs::msg::MotionCommand, pivot_command),  // bytes offset in struct
    nullptr,  // default value
    size_function__MotionCommand__pivot_command,  // size() function pointer
    get_const_function__MotionCommand__pivot_command,  // get_const(index) function pointer
    get_function__MotionCommand__pivot_command,  // get(index) function pointer
    resize_function__MotionCommand__pivot_command  // resize(index) function pointer
  },
  {
    "settle_command",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<rj_msgs::msg::SettleMotionCommand>(),  // members of sub message
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(rj_msgs::msg::MotionCommand, settle_command),  // bytes offset in struct
    nullptr,  // default value
    size_function__MotionCommand__settle_command,  // size() function pointer
    get_const_function__MotionCommand__settle_command,  // get_const(index) function pointer
    get_function__MotionCommand__settle_command,  // get(index) function pointer
    resize_function__MotionCommand__settle_command  // resize(index) function pointer
  },
  {
    "collect_command",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<rj_msgs::msg::CollectMotionCommand>(),  // members of sub message
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(rj_msgs::msg::MotionCommand, collect_command),  // bytes offset in struct
    nullptr,  // default value
    size_function__MotionCommand__collect_command,  // size() function pointer
    get_const_function__MotionCommand__collect_command,  // get_const(index) function pointer
    get_function__MotionCommand__collect_command,  // get(index) function pointer
    resize_function__MotionCommand__collect_command  // resize(index) function pointer
  },
  {
    "line_kick_command",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<rj_msgs::msg::LineKickMotionCommand>(),  // members of sub message
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(rj_msgs::msg::MotionCommand, line_kick_command),  // bytes offset in struct
    nullptr,  // default value
    size_function__MotionCommand__line_kick_command,  // size() function pointer
    get_const_function__MotionCommand__line_kick_command,  // get_const(index) function pointer
    get_function__MotionCommand__line_kick_command,  // get(index) function pointer
    resize_function__MotionCommand__line_kick_command  // resize(index) function pointer
  },
  {
    "intercept_command",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<rj_msgs::msg::InterceptMotionCommand>(),  // members of sub message
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(rj_msgs::msg::MotionCommand, intercept_command),  // bytes offset in struct
    nullptr,  // default value
    size_function__MotionCommand__intercept_command,  // size() function pointer
    get_const_function__MotionCommand__intercept_command,  // get_const(index) function pointer
    get_function__MotionCommand__intercept_command,  // get(index) function pointer
    resize_function__MotionCommand__intercept_command  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers MotionCommand_message_members = {
  "rj_msgs::msg",  // message namespace
  "MotionCommand",  // message name
  8,  // number of fields
  sizeof(rj_msgs::msg::MotionCommand),
  MotionCommand_message_member_array,  // message members
  MotionCommand_init_function,  // function to initialize message memory (memory has to be allocated)
  MotionCommand_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t MotionCommand_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &MotionCommand_message_members,
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
get_message_type_support_handle<rj_msgs::msg::MotionCommand>()
{
  return &::rj_msgs::msg::rosidl_typesupport_introspection_cpp::MotionCommand_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, rj_msgs, msg, MotionCommand)() {
  return &::rj_msgs::msg::rosidl_typesupport_introspection_cpp::MotionCommand_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
