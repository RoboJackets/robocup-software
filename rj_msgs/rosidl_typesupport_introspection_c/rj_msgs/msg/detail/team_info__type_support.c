// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from rj_msgs:msg/TeamInfo.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "rj_msgs/msg/detail/team_info__rosidl_typesupport_introspection_c.h"
#include "rj_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "rj_msgs/msg/detail/team_info__functions.h"
#include "rj_msgs/msg/detail/team_info__struct.h"


// Include directives for member types
// Member `name`
#include "rosidl_runtime_c/string_functions.h"
// Member `yellow_card_remaining_times`
// Member `remaining_timeout_time`
#include "builtin_interfaces/msg/duration.h"
// Member `yellow_card_remaining_times`
// Member `remaining_timeout_time`
#include "builtin_interfaces/msg/detail/duration__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void TeamInfo__rosidl_typesupport_introspection_c__TeamInfo_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  rj_msgs__msg__TeamInfo__init(message_memory);
}

void TeamInfo__rosidl_typesupport_introspection_c__TeamInfo_fini_function(void * message_memory)
{
  rj_msgs__msg__TeamInfo__fini(message_memory);
}

size_t TeamInfo__rosidl_typesupport_introspection_c__size_function__Duration__yellow_card_remaining_times(
  const void * untyped_member)
{
  const builtin_interfaces__msg__Duration__Sequence * member =
    (const builtin_interfaces__msg__Duration__Sequence *)(untyped_member);
  return member->size;
}

const void * TeamInfo__rosidl_typesupport_introspection_c__get_const_function__Duration__yellow_card_remaining_times(
  const void * untyped_member, size_t index)
{
  const builtin_interfaces__msg__Duration__Sequence * member =
    (const builtin_interfaces__msg__Duration__Sequence *)(untyped_member);
  return &member->data[index];
}

void * TeamInfo__rosidl_typesupport_introspection_c__get_function__Duration__yellow_card_remaining_times(
  void * untyped_member, size_t index)
{
  builtin_interfaces__msg__Duration__Sequence * member =
    (builtin_interfaces__msg__Duration__Sequence *)(untyped_member);
  return &member->data[index];
}

bool TeamInfo__rosidl_typesupport_introspection_c__resize_function__Duration__yellow_card_remaining_times(
  void * untyped_member, size_t size)
{
  builtin_interfaces__msg__Duration__Sequence * member =
    (builtin_interfaces__msg__Duration__Sequence *)(untyped_member);
  builtin_interfaces__msg__Duration__Sequence__fini(member);
  return builtin_interfaces__msg__Duration__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember TeamInfo__rosidl_typesupport_introspection_c__TeamInfo_message_member_array[8] = {
  {
    "name",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rj_msgs__msg__TeamInfo, name),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "score",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rj_msgs__msg__TeamInfo, score),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "num_red_cards",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rj_msgs__msg__TeamInfo, num_red_cards),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "num_yellow_cards",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rj_msgs__msg__TeamInfo, num_yellow_cards),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "yellow_card_remaining_times",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rj_msgs__msg__TeamInfo, yellow_card_remaining_times),  // bytes offset in struct
    NULL,  // default value
    TeamInfo__rosidl_typesupport_introspection_c__size_function__Duration__yellow_card_remaining_times,  // size() function pointer
    TeamInfo__rosidl_typesupport_introspection_c__get_const_function__Duration__yellow_card_remaining_times,  // get_const(index) function pointer
    TeamInfo__rosidl_typesupport_introspection_c__get_function__Duration__yellow_card_remaining_times,  // get(index) function pointer
    TeamInfo__rosidl_typesupport_introspection_c__resize_function__Duration__yellow_card_remaining_times  // resize(index) function pointer
  },
  {
    "timeouts_left",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rj_msgs__msg__TeamInfo, timeouts_left),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "remaining_timeout_time",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rj_msgs__msg__TeamInfo, remaining_timeout_time),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "goalie_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rj_msgs__msg__TeamInfo, goalie_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers TeamInfo__rosidl_typesupport_introspection_c__TeamInfo_message_members = {
  "rj_msgs__msg",  // message namespace
  "TeamInfo",  // message name
  8,  // number of fields
  sizeof(rj_msgs__msg__TeamInfo),
  TeamInfo__rosidl_typesupport_introspection_c__TeamInfo_message_member_array,  // message members
  TeamInfo__rosidl_typesupport_introspection_c__TeamInfo_init_function,  // function to initialize message memory (memory has to be allocated)
  TeamInfo__rosidl_typesupport_introspection_c__TeamInfo_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t TeamInfo__rosidl_typesupport_introspection_c__TeamInfo_message_type_support_handle = {
  0,
  &TeamInfo__rosidl_typesupport_introspection_c__TeamInfo_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_rj_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rj_msgs, msg, TeamInfo)() {
  TeamInfo__rosidl_typesupport_introspection_c__TeamInfo_message_member_array[4].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, builtin_interfaces, msg, Duration)();
  TeamInfo__rosidl_typesupport_introspection_c__TeamInfo_message_member_array[6].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, builtin_interfaces, msg, Duration)();
  if (!TeamInfo__rosidl_typesupport_introspection_c__TeamInfo_message_type_support_handle.typesupport_identifier) {
    TeamInfo__rosidl_typesupport_introspection_c__TeamInfo_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &TeamInfo__rosidl_typesupport_introspection_c__TeamInfo_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
