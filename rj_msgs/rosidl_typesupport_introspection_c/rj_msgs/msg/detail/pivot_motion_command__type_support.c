// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from rj_msgs:msg/PivotMotionCommand.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "rj_msgs/msg/detail/pivot_motion_command__rosidl_typesupport_introspection_c.h"
#include "rj_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "rj_msgs/msg/detail/pivot_motion_command__functions.h"
#include "rj_msgs/msg/detail/pivot_motion_command__struct.h"


// Include directives for member types
// Member `pivot_point`
// Member `pivot_target`
#include "rj_geometry_msgs/msg/point.h"
// Member `pivot_point`
// Member `pivot_target`
#include "rj_geometry_msgs/msg/detail/point__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void PivotMotionCommand__rosidl_typesupport_introspection_c__PivotMotionCommand_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  rj_msgs__msg__PivotMotionCommand__init(message_memory);
}

void PivotMotionCommand__rosidl_typesupport_introspection_c__PivotMotionCommand_fini_function(void * message_memory)
{
  rj_msgs__msg__PivotMotionCommand__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember PivotMotionCommand__rosidl_typesupport_introspection_c__PivotMotionCommand_message_member_array[2] = {
  {
    "pivot_point",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rj_msgs__msg__PivotMotionCommand, pivot_point),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "pivot_target",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rj_msgs__msg__PivotMotionCommand, pivot_target),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers PivotMotionCommand__rosidl_typesupport_introspection_c__PivotMotionCommand_message_members = {
  "rj_msgs__msg",  // message namespace
  "PivotMotionCommand",  // message name
  2,  // number of fields
  sizeof(rj_msgs__msg__PivotMotionCommand),
  PivotMotionCommand__rosidl_typesupport_introspection_c__PivotMotionCommand_message_member_array,  // message members
  PivotMotionCommand__rosidl_typesupport_introspection_c__PivotMotionCommand_init_function,  // function to initialize message memory (memory has to be allocated)
  PivotMotionCommand__rosidl_typesupport_introspection_c__PivotMotionCommand_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t PivotMotionCommand__rosidl_typesupport_introspection_c__PivotMotionCommand_message_type_support_handle = {
  0,
  &PivotMotionCommand__rosidl_typesupport_introspection_c__PivotMotionCommand_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_rj_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rj_msgs, msg, PivotMotionCommand)() {
  PivotMotionCommand__rosidl_typesupport_introspection_c__PivotMotionCommand_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rj_geometry_msgs, msg, Point)();
  PivotMotionCommand__rosidl_typesupport_introspection_c__PivotMotionCommand_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rj_geometry_msgs, msg, Point)();
  if (!PivotMotionCommand__rosidl_typesupport_introspection_c__PivotMotionCommand_message_type_support_handle.typesupport_identifier) {
    PivotMotionCommand__rosidl_typesupport_introspection_c__PivotMotionCommand_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &PivotMotionCommand__rosidl_typesupport_introspection_c__PivotMotionCommand_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
