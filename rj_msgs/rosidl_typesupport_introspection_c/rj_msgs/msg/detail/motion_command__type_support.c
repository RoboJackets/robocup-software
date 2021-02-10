// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from rj_msgs:msg/MotionCommand.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "rj_msgs/msg/detail/motion_command__rosidl_typesupport_introspection_c.h"
#include "rj_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "rj_msgs/msg/detail/motion_command__functions.h"
#include "rj_msgs/msg/detail/motion_command__struct.h"


// Include directives for member types
// Member `empty_command`
#include "rj_msgs/msg/empty_motion_command.h"
// Member `empty_command`
#include "rj_msgs/msg/detail/empty_motion_command__rosidl_typesupport_introspection_c.h"
// Member `path_target_command`
#include "rj_msgs/msg/path_target_motion_command.h"
// Member `path_target_command`
#include "rj_msgs/msg/detail/path_target_motion_command__rosidl_typesupport_introspection_c.h"
// Member `world_vel_command`
#include "rj_msgs/msg/world_vel_motion_command.h"
// Member `world_vel_command`
#include "rj_msgs/msg/detail/world_vel_motion_command__rosidl_typesupport_introspection_c.h"
// Member `pivot_command`
#include "rj_msgs/msg/pivot_motion_command.h"
// Member `pivot_command`
#include "rj_msgs/msg/detail/pivot_motion_command__rosidl_typesupport_introspection_c.h"
// Member `settle_command`
#include "rj_msgs/msg/settle_motion_command.h"
// Member `settle_command`
#include "rj_msgs/msg/detail/settle_motion_command__rosidl_typesupport_introspection_c.h"
// Member `collect_command`
#include "rj_msgs/msg/collect_motion_command.h"
// Member `collect_command`
#include "rj_msgs/msg/detail/collect_motion_command__rosidl_typesupport_introspection_c.h"
// Member `line_kick_command`
#include "rj_msgs/msg/line_kick_motion_command.h"
// Member `line_kick_command`
#include "rj_msgs/msg/detail/line_kick_motion_command__rosidl_typesupport_introspection_c.h"
// Member `intercept_command`
#include "rj_msgs/msg/intercept_motion_command.h"
// Member `intercept_command`
#include "rj_msgs/msg/detail/intercept_motion_command__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void MotionCommand__rosidl_typesupport_introspection_c__MotionCommand_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  rj_msgs__msg__MotionCommand__init(message_memory);
}

void MotionCommand__rosidl_typesupport_introspection_c__MotionCommand_fini_function(void * message_memory)
{
  rj_msgs__msg__MotionCommand__fini(message_memory);
}

size_t MotionCommand__rosidl_typesupport_introspection_c__size_function__EmptyMotionCommand__empty_command(
  const void * untyped_member)
{
  const rj_msgs__msg__EmptyMotionCommand__Sequence * member =
    (const rj_msgs__msg__EmptyMotionCommand__Sequence *)(untyped_member);
  return member->size;
}

const void * MotionCommand__rosidl_typesupport_introspection_c__get_const_function__EmptyMotionCommand__empty_command(
  const void * untyped_member, size_t index)
{
  const rj_msgs__msg__EmptyMotionCommand__Sequence * member =
    (const rj_msgs__msg__EmptyMotionCommand__Sequence *)(untyped_member);
  return &member->data[index];
}

void * MotionCommand__rosidl_typesupport_introspection_c__get_function__EmptyMotionCommand__empty_command(
  void * untyped_member, size_t index)
{
  rj_msgs__msg__EmptyMotionCommand__Sequence * member =
    (rj_msgs__msg__EmptyMotionCommand__Sequence *)(untyped_member);
  return &member->data[index];
}

bool MotionCommand__rosidl_typesupport_introspection_c__resize_function__EmptyMotionCommand__empty_command(
  void * untyped_member, size_t size)
{
  rj_msgs__msg__EmptyMotionCommand__Sequence * member =
    (rj_msgs__msg__EmptyMotionCommand__Sequence *)(untyped_member);
  rj_msgs__msg__EmptyMotionCommand__Sequence__fini(member);
  return rj_msgs__msg__EmptyMotionCommand__Sequence__init(member, size);
}

size_t MotionCommand__rosidl_typesupport_introspection_c__size_function__PathTargetMotionCommand__path_target_command(
  const void * untyped_member)
{
  const rj_msgs__msg__PathTargetMotionCommand__Sequence * member =
    (const rj_msgs__msg__PathTargetMotionCommand__Sequence *)(untyped_member);
  return member->size;
}

const void * MotionCommand__rosidl_typesupport_introspection_c__get_const_function__PathTargetMotionCommand__path_target_command(
  const void * untyped_member, size_t index)
{
  const rj_msgs__msg__PathTargetMotionCommand__Sequence * member =
    (const rj_msgs__msg__PathTargetMotionCommand__Sequence *)(untyped_member);
  return &member->data[index];
}

void * MotionCommand__rosidl_typesupport_introspection_c__get_function__PathTargetMotionCommand__path_target_command(
  void * untyped_member, size_t index)
{
  rj_msgs__msg__PathTargetMotionCommand__Sequence * member =
    (rj_msgs__msg__PathTargetMotionCommand__Sequence *)(untyped_member);
  return &member->data[index];
}

bool MotionCommand__rosidl_typesupport_introspection_c__resize_function__PathTargetMotionCommand__path_target_command(
  void * untyped_member, size_t size)
{
  rj_msgs__msg__PathTargetMotionCommand__Sequence * member =
    (rj_msgs__msg__PathTargetMotionCommand__Sequence *)(untyped_member);
  rj_msgs__msg__PathTargetMotionCommand__Sequence__fini(member);
  return rj_msgs__msg__PathTargetMotionCommand__Sequence__init(member, size);
}

size_t MotionCommand__rosidl_typesupport_introspection_c__size_function__WorldVelMotionCommand__world_vel_command(
  const void * untyped_member)
{
  const rj_msgs__msg__WorldVelMotionCommand__Sequence * member =
    (const rj_msgs__msg__WorldVelMotionCommand__Sequence *)(untyped_member);
  return member->size;
}

const void * MotionCommand__rosidl_typesupport_introspection_c__get_const_function__WorldVelMotionCommand__world_vel_command(
  const void * untyped_member, size_t index)
{
  const rj_msgs__msg__WorldVelMotionCommand__Sequence * member =
    (const rj_msgs__msg__WorldVelMotionCommand__Sequence *)(untyped_member);
  return &member->data[index];
}

void * MotionCommand__rosidl_typesupport_introspection_c__get_function__WorldVelMotionCommand__world_vel_command(
  void * untyped_member, size_t index)
{
  rj_msgs__msg__WorldVelMotionCommand__Sequence * member =
    (rj_msgs__msg__WorldVelMotionCommand__Sequence *)(untyped_member);
  return &member->data[index];
}

bool MotionCommand__rosidl_typesupport_introspection_c__resize_function__WorldVelMotionCommand__world_vel_command(
  void * untyped_member, size_t size)
{
  rj_msgs__msg__WorldVelMotionCommand__Sequence * member =
    (rj_msgs__msg__WorldVelMotionCommand__Sequence *)(untyped_member);
  rj_msgs__msg__WorldVelMotionCommand__Sequence__fini(member);
  return rj_msgs__msg__WorldVelMotionCommand__Sequence__init(member, size);
}

size_t MotionCommand__rosidl_typesupport_introspection_c__size_function__PivotMotionCommand__pivot_command(
  const void * untyped_member)
{
  const rj_msgs__msg__PivotMotionCommand__Sequence * member =
    (const rj_msgs__msg__PivotMotionCommand__Sequence *)(untyped_member);
  return member->size;
}

const void * MotionCommand__rosidl_typesupport_introspection_c__get_const_function__PivotMotionCommand__pivot_command(
  const void * untyped_member, size_t index)
{
  const rj_msgs__msg__PivotMotionCommand__Sequence * member =
    (const rj_msgs__msg__PivotMotionCommand__Sequence *)(untyped_member);
  return &member->data[index];
}

void * MotionCommand__rosidl_typesupport_introspection_c__get_function__PivotMotionCommand__pivot_command(
  void * untyped_member, size_t index)
{
  rj_msgs__msg__PivotMotionCommand__Sequence * member =
    (rj_msgs__msg__PivotMotionCommand__Sequence *)(untyped_member);
  return &member->data[index];
}

bool MotionCommand__rosidl_typesupport_introspection_c__resize_function__PivotMotionCommand__pivot_command(
  void * untyped_member, size_t size)
{
  rj_msgs__msg__PivotMotionCommand__Sequence * member =
    (rj_msgs__msg__PivotMotionCommand__Sequence *)(untyped_member);
  rj_msgs__msg__PivotMotionCommand__Sequence__fini(member);
  return rj_msgs__msg__PivotMotionCommand__Sequence__init(member, size);
}

size_t MotionCommand__rosidl_typesupport_introspection_c__size_function__SettleMotionCommand__settle_command(
  const void * untyped_member)
{
  const rj_msgs__msg__SettleMotionCommand__Sequence * member =
    (const rj_msgs__msg__SettleMotionCommand__Sequence *)(untyped_member);
  return member->size;
}

const void * MotionCommand__rosidl_typesupport_introspection_c__get_const_function__SettleMotionCommand__settle_command(
  const void * untyped_member, size_t index)
{
  const rj_msgs__msg__SettleMotionCommand__Sequence * member =
    (const rj_msgs__msg__SettleMotionCommand__Sequence *)(untyped_member);
  return &member->data[index];
}

void * MotionCommand__rosidl_typesupport_introspection_c__get_function__SettleMotionCommand__settle_command(
  void * untyped_member, size_t index)
{
  rj_msgs__msg__SettleMotionCommand__Sequence * member =
    (rj_msgs__msg__SettleMotionCommand__Sequence *)(untyped_member);
  return &member->data[index];
}

bool MotionCommand__rosidl_typesupport_introspection_c__resize_function__SettleMotionCommand__settle_command(
  void * untyped_member, size_t size)
{
  rj_msgs__msg__SettleMotionCommand__Sequence * member =
    (rj_msgs__msg__SettleMotionCommand__Sequence *)(untyped_member);
  rj_msgs__msg__SettleMotionCommand__Sequence__fini(member);
  return rj_msgs__msg__SettleMotionCommand__Sequence__init(member, size);
}

size_t MotionCommand__rosidl_typesupport_introspection_c__size_function__CollectMotionCommand__collect_command(
  const void * untyped_member)
{
  const rj_msgs__msg__CollectMotionCommand__Sequence * member =
    (const rj_msgs__msg__CollectMotionCommand__Sequence *)(untyped_member);
  return member->size;
}

const void * MotionCommand__rosidl_typesupport_introspection_c__get_const_function__CollectMotionCommand__collect_command(
  const void * untyped_member, size_t index)
{
  const rj_msgs__msg__CollectMotionCommand__Sequence * member =
    (const rj_msgs__msg__CollectMotionCommand__Sequence *)(untyped_member);
  return &member->data[index];
}

void * MotionCommand__rosidl_typesupport_introspection_c__get_function__CollectMotionCommand__collect_command(
  void * untyped_member, size_t index)
{
  rj_msgs__msg__CollectMotionCommand__Sequence * member =
    (rj_msgs__msg__CollectMotionCommand__Sequence *)(untyped_member);
  return &member->data[index];
}

bool MotionCommand__rosidl_typesupport_introspection_c__resize_function__CollectMotionCommand__collect_command(
  void * untyped_member, size_t size)
{
  rj_msgs__msg__CollectMotionCommand__Sequence * member =
    (rj_msgs__msg__CollectMotionCommand__Sequence *)(untyped_member);
  rj_msgs__msg__CollectMotionCommand__Sequence__fini(member);
  return rj_msgs__msg__CollectMotionCommand__Sequence__init(member, size);
}

size_t MotionCommand__rosidl_typesupport_introspection_c__size_function__LineKickMotionCommand__line_kick_command(
  const void * untyped_member)
{
  const rj_msgs__msg__LineKickMotionCommand__Sequence * member =
    (const rj_msgs__msg__LineKickMotionCommand__Sequence *)(untyped_member);
  return member->size;
}

const void * MotionCommand__rosidl_typesupport_introspection_c__get_const_function__LineKickMotionCommand__line_kick_command(
  const void * untyped_member, size_t index)
{
  const rj_msgs__msg__LineKickMotionCommand__Sequence * member =
    (const rj_msgs__msg__LineKickMotionCommand__Sequence *)(untyped_member);
  return &member->data[index];
}

void * MotionCommand__rosidl_typesupport_introspection_c__get_function__LineKickMotionCommand__line_kick_command(
  void * untyped_member, size_t index)
{
  rj_msgs__msg__LineKickMotionCommand__Sequence * member =
    (rj_msgs__msg__LineKickMotionCommand__Sequence *)(untyped_member);
  return &member->data[index];
}

bool MotionCommand__rosidl_typesupport_introspection_c__resize_function__LineKickMotionCommand__line_kick_command(
  void * untyped_member, size_t size)
{
  rj_msgs__msg__LineKickMotionCommand__Sequence * member =
    (rj_msgs__msg__LineKickMotionCommand__Sequence *)(untyped_member);
  rj_msgs__msg__LineKickMotionCommand__Sequence__fini(member);
  return rj_msgs__msg__LineKickMotionCommand__Sequence__init(member, size);
}

size_t MotionCommand__rosidl_typesupport_introspection_c__size_function__InterceptMotionCommand__intercept_command(
  const void * untyped_member)
{
  const rj_msgs__msg__InterceptMotionCommand__Sequence * member =
    (const rj_msgs__msg__InterceptMotionCommand__Sequence *)(untyped_member);
  return member->size;
}

const void * MotionCommand__rosidl_typesupport_introspection_c__get_const_function__InterceptMotionCommand__intercept_command(
  const void * untyped_member, size_t index)
{
  const rj_msgs__msg__InterceptMotionCommand__Sequence * member =
    (const rj_msgs__msg__InterceptMotionCommand__Sequence *)(untyped_member);
  return &member->data[index];
}

void * MotionCommand__rosidl_typesupport_introspection_c__get_function__InterceptMotionCommand__intercept_command(
  void * untyped_member, size_t index)
{
  rj_msgs__msg__InterceptMotionCommand__Sequence * member =
    (rj_msgs__msg__InterceptMotionCommand__Sequence *)(untyped_member);
  return &member->data[index];
}

bool MotionCommand__rosidl_typesupport_introspection_c__resize_function__InterceptMotionCommand__intercept_command(
  void * untyped_member, size_t size)
{
  rj_msgs__msg__InterceptMotionCommand__Sequence * member =
    (rj_msgs__msg__InterceptMotionCommand__Sequence *)(untyped_member);
  rj_msgs__msg__InterceptMotionCommand__Sequence__fini(member);
  return rj_msgs__msg__InterceptMotionCommand__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember MotionCommand__rosidl_typesupport_introspection_c__MotionCommand_message_member_array[8] = {
  {
    "empty_command",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(rj_msgs__msg__MotionCommand, empty_command),  // bytes offset in struct
    NULL,  // default value
    MotionCommand__rosidl_typesupport_introspection_c__size_function__EmptyMotionCommand__empty_command,  // size() function pointer
    MotionCommand__rosidl_typesupport_introspection_c__get_const_function__EmptyMotionCommand__empty_command,  // get_const(index) function pointer
    MotionCommand__rosidl_typesupport_introspection_c__get_function__EmptyMotionCommand__empty_command,  // get(index) function pointer
    MotionCommand__rosidl_typesupport_introspection_c__resize_function__EmptyMotionCommand__empty_command  // resize(index) function pointer
  },
  {
    "path_target_command",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(rj_msgs__msg__MotionCommand, path_target_command),  // bytes offset in struct
    NULL,  // default value
    MotionCommand__rosidl_typesupport_introspection_c__size_function__PathTargetMotionCommand__path_target_command,  // size() function pointer
    MotionCommand__rosidl_typesupport_introspection_c__get_const_function__PathTargetMotionCommand__path_target_command,  // get_const(index) function pointer
    MotionCommand__rosidl_typesupport_introspection_c__get_function__PathTargetMotionCommand__path_target_command,  // get(index) function pointer
    MotionCommand__rosidl_typesupport_introspection_c__resize_function__PathTargetMotionCommand__path_target_command  // resize(index) function pointer
  },
  {
    "world_vel_command",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(rj_msgs__msg__MotionCommand, world_vel_command),  // bytes offset in struct
    NULL,  // default value
    MotionCommand__rosidl_typesupport_introspection_c__size_function__WorldVelMotionCommand__world_vel_command,  // size() function pointer
    MotionCommand__rosidl_typesupport_introspection_c__get_const_function__WorldVelMotionCommand__world_vel_command,  // get_const(index) function pointer
    MotionCommand__rosidl_typesupport_introspection_c__get_function__WorldVelMotionCommand__world_vel_command,  // get(index) function pointer
    MotionCommand__rosidl_typesupport_introspection_c__resize_function__WorldVelMotionCommand__world_vel_command  // resize(index) function pointer
  },
  {
    "pivot_command",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(rj_msgs__msg__MotionCommand, pivot_command),  // bytes offset in struct
    NULL,  // default value
    MotionCommand__rosidl_typesupport_introspection_c__size_function__PivotMotionCommand__pivot_command,  // size() function pointer
    MotionCommand__rosidl_typesupport_introspection_c__get_const_function__PivotMotionCommand__pivot_command,  // get_const(index) function pointer
    MotionCommand__rosidl_typesupport_introspection_c__get_function__PivotMotionCommand__pivot_command,  // get(index) function pointer
    MotionCommand__rosidl_typesupport_introspection_c__resize_function__PivotMotionCommand__pivot_command  // resize(index) function pointer
  },
  {
    "settle_command",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(rj_msgs__msg__MotionCommand, settle_command),  // bytes offset in struct
    NULL,  // default value
    MotionCommand__rosidl_typesupport_introspection_c__size_function__SettleMotionCommand__settle_command,  // size() function pointer
    MotionCommand__rosidl_typesupport_introspection_c__get_const_function__SettleMotionCommand__settle_command,  // get_const(index) function pointer
    MotionCommand__rosidl_typesupport_introspection_c__get_function__SettleMotionCommand__settle_command,  // get(index) function pointer
    MotionCommand__rosidl_typesupport_introspection_c__resize_function__SettleMotionCommand__settle_command  // resize(index) function pointer
  },
  {
    "collect_command",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(rj_msgs__msg__MotionCommand, collect_command),  // bytes offset in struct
    NULL,  // default value
    MotionCommand__rosidl_typesupport_introspection_c__size_function__CollectMotionCommand__collect_command,  // size() function pointer
    MotionCommand__rosidl_typesupport_introspection_c__get_const_function__CollectMotionCommand__collect_command,  // get_const(index) function pointer
    MotionCommand__rosidl_typesupport_introspection_c__get_function__CollectMotionCommand__collect_command,  // get(index) function pointer
    MotionCommand__rosidl_typesupport_introspection_c__resize_function__CollectMotionCommand__collect_command  // resize(index) function pointer
  },
  {
    "line_kick_command",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(rj_msgs__msg__MotionCommand, line_kick_command),  // bytes offset in struct
    NULL,  // default value
    MotionCommand__rosidl_typesupport_introspection_c__size_function__LineKickMotionCommand__line_kick_command,  // size() function pointer
    MotionCommand__rosidl_typesupport_introspection_c__get_const_function__LineKickMotionCommand__line_kick_command,  // get_const(index) function pointer
    MotionCommand__rosidl_typesupport_introspection_c__get_function__LineKickMotionCommand__line_kick_command,  // get(index) function pointer
    MotionCommand__rosidl_typesupport_introspection_c__resize_function__LineKickMotionCommand__line_kick_command  // resize(index) function pointer
  },
  {
    "intercept_command",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(rj_msgs__msg__MotionCommand, intercept_command),  // bytes offset in struct
    NULL,  // default value
    MotionCommand__rosidl_typesupport_introspection_c__size_function__InterceptMotionCommand__intercept_command,  // size() function pointer
    MotionCommand__rosidl_typesupport_introspection_c__get_const_function__InterceptMotionCommand__intercept_command,  // get_const(index) function pointer
    MotionCommand__rosidl_typesupport_introspection_c__get_function__InterceptMotionCommand__intercept_command,  // get(index) function pointer
    MotionCommand__rosidl_typesupport_introspection_c__resize_function__InterceptMotionCommand__intercept_command  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers MotionCommand__rosidl_typesupport_introspection_c__MotionCommand_message_members = {
  "rj_msgs__msg",  // message namespace
  "MotionCommand",  // message name
  8,  // number of fields
  sizeof(rj_msgs__msg__MotionCommand),
  MotionCommand__rosidl_typesupport_introspection_c__MotionCommand_message_member_array,  // message members
  MotionCommand__rosidl_typesupport_introspection_c__MotionCommand_init_function,  // function to initialize message memory (memory has to be allocated)
  MotionCommand__rosidl_typesupport_introspection_c__MotionCommand_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t MotionCommand__rosidl_typesupport_introspection_c__MotionCommand_message_type_support_handle = {
  0,
  &MotionCommand__rosidl_typesupport_introspection_c__MotionCommand_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_rj_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rj_msgs, msg, MotionCommand)() {
  MotionCommand__rosidl_typesupport_introspection_c__MotionCommand_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rj_msgs, msg, EmptyMotionCommand)();
  MotionCommand__rosidl_typesupport_introspection_c__MotionCommand_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rj_msgs, msg, PathTargetMotionCommand)();
  MotionCommand__rosidl_typesupport_introspection_c__MotionCommand_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rj_msgs, msg, WorldVelMotionCommand)();
  MotionCommand__rosidl_typesupport_introspection_c__MotionCommand_message_member_array[3].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rj_msgs, msg, PivotMotionCommand)();
  MotionCommand__rosidl_typesupport_introspection_c__MotionCommand_message_member_array[4].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rj_msgs, msg, SettleMotionCommand)();
  MotionCommand__rosidl_typesupport_introspection_c__MotionCommand_message_member_array[5].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rj_msgs, msg, CollectMotionCommand)();
  MotionCommand__rosidl_typesupport_introspection_c__MotionCommand_message_member_array[6].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rj_msgs, msg, LineKickMotionCommand)();
  MotionCommand__rosidl_typesupport_introspection_c__MotionCommand_message_member_array[7].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rj_msgs, msg, InterceptMotionCommand)();
  if (!MotionCommand__rosidl_typesupport_introspection_c__MotionCommand_message_type_support_handle.typesupport_identifier) {
    MotionCommand__rosidl_typesupport_introspection_c__MotionCommand_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &MotionCommand__rosidl_typesupport_introspection_c__MotionCommand_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
