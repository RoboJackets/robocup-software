// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from rj_msgs:msg/MotionSetpoint.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "rj_msgs/msg/detail/motion_setpoint__rosidl_typesupport_introspection_c.h"
#include "rj_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "rj_msgs/msg/detail/motion_setpoint__functions.h"
#include "rj_msgs/msg/detail/motion_setpoint__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void MotionSetpoint__rosidl_typesupport_introspection_c__MotionSetpoint_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  rj_msgs__msg__MotionSetpoint__init(message_memory);
}

void MotionSetpoint__rosidl_typesupport_introspection_c__MotionSetpoint_fini_function(void * message_memory)
{
  rj_msgs__msg__MotionSetpoint__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember MotionSetpoint__rosidl_typesupport_introspection_c__MotionSetpoint_message_member_array[3] = {
  {
    "velocity_x_mps",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rj_msgs__msg__MotionSetpoint, velocity_x_mps),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "velocity_y_mps",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rj_msgs__msg__MotionSetpoint, velocity_y_mps),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "velocity_z_radps",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rj_msgs__msg__MotionSetpoint, velocity_z_radps),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers MotionSetpoint__rosidl_typesupport_introspection_c__MotionSetpoint_message_members = {
  "rj_msgs__msg",  // message namespace
  "MotionSetpoint",  // message name
  3,  // number of fields
  sizeof(rj_msgs__msg__MotionSetpoint),
  MotionSetpoint__rosidl_typesupport_introspection_c__MotionSetpoint_message_member_array,  // message members
  MotionSetpoint__rosidl_typesupport_introspection_c__MotionSetpoint_init_function,  // function to initialize message memory (memory has to be allocated)
  MotionSetpoint__rosidl_typesupport_introspection_c__MotionSetpoint_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t MotionSetpoint__rosidl_typesupport_introspection_c__MotionSetpoint_message_type_support_handle = {
  0,
  &MotionSetpoint__rosidl_typesupport_introspection_c__MotionSetpoint_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_rj_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rj_msgs, msg, MotionSetpoint)() {
  if (!MotionSetpoint__rosidl_typesupport_introspection_c__MotionSetpoint_message_type_support_handle.typesupport_identifier) {
    MotionSetpoint__rosidl_typesupport_introspection_c__MotionSetpoint_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &MotionSetpoint__rosidl_typesupport_introspection_c__MotionSetpoint_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
