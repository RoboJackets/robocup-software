// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from rj_msgs:msg/SettleMotionCommand.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "rj_msgs/msg/detail/settle_motion_command__rosidl_typesupport_introspection_c.h"
#include "rj_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "rj_msgs/msg/detail/settle_motion_command__functions.h"
#include "rj_msgs/msg/detail/settle_motion_command__struct.h"


// Include directives for member types
// Member `maybe_target`
#include "rj_geometry_msgs/msg/point.h"
// Member `maybe_target`
#include "rj_geometry_msgs/msg/detail/point__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void SettleMotionCommand__rosidl_typesupport_introspection_c__SettleMotionCommand_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  rj_msgs__msg__SettleMotionCommand__init(message_memory);
}

void SettleMotionCommand__rosidl_typesupport_introspection_c__SettleMotionCommand_fini_function(void * message_memory)
{
  rj_msgs__msg__SettleMotionCommand__fini(message_memory);
}

size_t SettleMotionCommand__rosidl_typesupport_introspection_c__size_function__Point__maybe_target(
  const void * untyped_member)
{
  const rj_geometry_msgs__msg__Point__Sequence * member =
    (const rj_geometry_msgs__msg__Point__Sequence *)(untyped_member);
  return member->size;
}

const void * SettleMotionCommand__rosidl_typesupport_introspection_c__get_const_function__Point__maybe_target(
  const void * untyped_member, size_t index)
{
  const rj_geometry_msgs__msg__Point__Sequence * member =
    (const rj_geometry_msgs__msg__Point__Sequence *)(untyped_member);
  return &member->data[index];
}

void * SettleMotionCommand__rosidl_typesupport_introspection_c__get_function__Point__maybe_target(
  void * untyped_member, size_t index)
{
  rj_geometry_msgs__msg__Point__Sequence * member =
    (rj_geometry_msgs__msg__Point__Sequence *)(untyped_member);
  return &member->data[index];
}

bool SettleMotionCommand__rosidl_typesupport_introspection_c__resize_function__Point__maybe_target(
  void * untyped_member, size_t size)
{
  rj_geometry_msgs__msg__Point__Sequence * member =
    (rj_geometry_msgs__msg__Point__Sequence *)(untyped_member);
  rj_geometry_msgs__msg__Point__Sequence__fini(member);
  return rj_geometry_msgs__msg__Point__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember SettleMotionCommand__rosidl_typesupport_introspection_c__SettleMotionCommand_message_member_array[1] = {
  {
    "maybe_target",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(rj_msgs__msg__SettleMotionCommand, maybe_target),  // bytes offset in struct
    NULL,  // default value
    SettleMotionCommand__rosidl_typesupport_introspection_c__size_function__Point__maybe_target,  // size() function pointer
    SettleMotionCommand__rosidl_typesupport_introspection_c__get_const_function__Point__maybe_target,  // get_const(index) function pointer
    SettleMotionCommand__rosidl_typesupport_introspection_c__get_function__Point__maybe_target,  // get(index) function pointer
    SettleMotionCommand__rosidl_typesupport_introspection_c__resize_function__Point__maybe_target  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers SettleMotionCommand__rosidl_typesupport_introspection_c__SettleMotionCommand_message_members = {
  "rj_msgs__msg",  // message namespace
  "SettleMotionCommand",  // message name
  1,  // number of fields
  sizeof(rj_msgs__msg__SettleMotionCommand),
  SettleMotionCommand__rosidl_typesupport_introspection_c__SettleMotionCommand_message_member_array,  // message members
  SettleMotionCommand__rosidl_typesupport_introspection_c__SettleMotionCommand_init_function,  // function to initialize message memory (memory has to be allocated)
  SettleMotionCommand__rosidl_typesupport_introspection_c__SettleMotionCommand_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t SettleMotionCommand__rosidl_typesupport_introspection_c__SettleMotionCommand_message_type_support_handle = {
  0,
  &SettleMotionCommand__rosidl_typesupport_introspection_c__SettleMotionCommand_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_rj_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rj_msgs, msg, SettleMotionCommand)() {
  SettleMotionCommand__rosidl_typesupport_introspection_c__SettleMotionCommand_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rj_geometry_msgs, msg, Point)();
  if (!SettleMotionCommand__rosidl_typesupport_introspection_c__SettleMotionCommand_message_type_support_handle.typesupport_identifier) {
    SettleMotionCommand__rosidl_typesupport_introspection_c__SettleMotionCommand_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &SettleMotionCommand__rosidl_typesupport_introspection_c__SettleMotionCommand_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
