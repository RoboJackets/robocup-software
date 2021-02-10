// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from rj_msgs:msg/WorldState.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "rj_msgs/msg/detail/world_state__rosidl_typesupport_introspection_c.h"
#include "rj_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "rj_msgs/msg/detail/world_state__functions.h"
#include "rj_msgs/msg/detail/world_state__struct.h"


// Include directives for member types
// Member `last_update_time`
#include "builtin_interfaces/msg/time.h"
// Member `last_update_time`
#include "builtin_interfaces/msg/detail/time__rosidl_typesupport_introspection_c.h"
// Member `their_robots`
// Member `our_robots`
#include "rj_msgs/msg/robot_state.h"
// Member `their_robots`
// Member `our_robots`
#include "rj_msgs/msg/detail/robot_state__rosidl_typesupport_introspection_c.h"
// Member `ball`
#include "rj_msgs/msg/ball_state.h"
// Member `ball`
#include "rj_msgs/msg/detail/ball_state__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void WorldState__rosidl_typesupport_introspection_c__WorldState_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  rj_msgs__msg__WorldState__init(message_memory);
}

void WorldState__rosidl_typesupport_introspection_c__WorldState_fini_function(void * message_memory)
{
  rj_msgs__msg__WorldState__fini(message_memory);
}

size_t WorldState__rosidl_typesupport_introspection_c__size_function__RobotState__their_robots(
  const void * untyped_member)
{
  const rj_msgs__msg__RobotState__Sequence * member =
    (const rj_msgs__msg__RobotState__Sequence *)(untyped_member);
  return member->size;
}

const void * WorldState__rosidl_typesupport_introspection_c__get_const_function__RobotState__their_robots(
  const void * untyped_member, size_t index)
{
  const rj_msgs__msg__RobotState__Sequence * member =
    (const rj_msgs__msg__RobotState__Sequence *)(untyped_member);
  return &member->data[index];
}

void * WorldState__rosidl_typesupport_introspection_c__get_function__RobotState__their_robots(
  void * untyped_member, size_t index)
{
  rj_msgs__msg__RobotState__Sequence * member =
    (rj_msgs__msg__RobotState__Sequence *)(untyped_member);
  return &member->data[index];
}

bool WorldState__rosidl_typesupport_introspection_c__resize_function__RobotState__their_robots(
  void * untyped_member, size_t size)
{
  rj_msgs__msg__RobotState__Sequence * member =
    (rj_msgs__msg__RobotState__Sequence *)(untyped_member);
  rj_msgs__msg__RobotState__Sequence__fini(member);
  return rj_msgs__msg__RobotState__Sequence__init(member, size);
}

size_t WorldState__rosidl_typesupport_introspection_c__size_function__RobotState__our_robots(
  const void * untyped_member)
{
  const rj_msgs__msg__RobotState__Sequence * member =
    (const rj_msgs__msg__RobotState__Sequence *)(untyped_member);
  return member->size;
}

const void * WorldState__rosidl_typesupport_introspection_c__get_const_function__RobotState__our_robots(
  const void * untyped_member, size_t index)
{
  const rj_msgs__msg__RobotState__Sequence * member =
    (const rj_msgs__msg__RobotState__Sequence *)(untyped_member);
  return &member->data[index];
}

void * WorldState__rosidl_typesupport_introspection_c__get_function__RobotState__our_robots(
  void * untyped_member, size_t index)
{
  rj_msgs__msg__RobotState__Sequence * member =
    (rj_msgs__msg__RobotState__Sequence *)(untyped_member);
  return &member->data[index];
}

bool WorldState__rosidl_typesupport_introspection_c__resize_function__RobotState__our_robots(
  void * untyped_member, size_t size)
{
  rj_msgs__msg__RobotState__Sequence * member =
    (rj_msgs__msg__RobotState__Sequence *)(untyped_member);
  rj_msgs__msg__RobotState__Sequence__fini(member);
  return rj_msgs__msg__RobotState__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember WorldState__rosidl_typesupport_introspection_c__WorldState_message_member_array[4] = {
  {
    "last_update_time",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rj_msgs__msg__WorldState, last_update_time),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "their_robots",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rj_msgs__msg__WorldState, their_robots),  // bytes offset in struct
    NULL,  // default value
    WorldState__rosidl_typesupport_introspection_c__size_function__RobotState__their_robots,  // size() function pointer
    WorldState__rosidl_typesupport_introspection_c__get_const_function__RobotState__their_robots,  // get_const(index) function pointer
    WorldState__rosidl_typesupport_introspection_c__get_function__RobotState__their_robots,  // get(index) function pointer
    WorldState__rosidl_typesupport_introspection_c__resize_function__RobotState__their_robots  // resize(index) function pointer
  },
  {
    "our_robots",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rj_msgs__msg__WorldState, our_robots),  // bytes offset in struct
    NULL,  // default value
    WorldState__rosidl_typesupport_introspection_c__size_function__RobotState__our_robots,  // size() function pointer
    WorldState__rosidl_typesupport_introspection_c__get_const_function__RobotState__our_robots,  // get_const(index) function pointer
    WorldState__rosidl_typesupport_introspection_c__get_function__RobotState__our_robots,  // get(index) function pointer
    WorldState__rosidl_typesupport_introspection_c__resize_function__RobotState__our_robots  // resize(index) function pointer
  },
  {
    "ball",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rj_msgs__msg__WorldState, ball),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers WorldState__rosidl_typesupport_introspection_c__WorldState_message_members = {
  "rj_msgs__msg",  // message namespace
  "WorldState",  // message name
  4,  // number of fields
  sizeof(rj_msgs__msg__WorldState),
  WorldState__rosidl_typesupport_introspection_c__WorldState_message_member_array,  // message members
  WorldState__rosidl_typesupport_introspection_c__WorldState_init_function,  // function to initialize message memory (memory has to be allocated)
  WorldState__rosidl_typesupport_introspection_c__WorldState_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t WorldState__rosidl_typesupport_introspection_c__WorldState_message_type_support_handle = {
  0,
  &WorldState__rosidl_typesupport_introspection_c__WorldState_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_rj_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rj_msgs, msg, WorldState)() {
  WorldState__rosidl_typesupport_introspection_c__WorldState_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, builtin_interfaces, msg, Time)();
  WorldState__rosidl_typesupport_introspection_c__WorldState_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rj_msgs, msg, RobotState)();
  WorldState__rosidl_typesupport_introspection_c__WorldState_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rj_msgs, msg, RobotState)();
  WorldState__rosidl_typesupport_introspection_c__WorldState_message_member_array[3].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rj_msgs, msg, BallState)();
  if (!WorldState__rosidl_typesupport_introspection_c__WorldState_message_type_support_handle.typesupport_identifier) {
    WorldState__rosidl_typesupport_introspection_c__WorldState_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &WorldState__rosidl_typesupport_introspection_c__WorldState_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
