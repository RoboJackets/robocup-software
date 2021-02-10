// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from rj_msgs:msg/Trajectory.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "rj_msgs/msg/detail/trajectory__rosidl_typesupport_introspection_c.h"
#include "rj_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "rj_msgs/msg/detail/trajectory__functions.h"
#include "rj_msgs/msg/detail/trajectory__struct.h"


// Include directives for member types
// Member `stamp`
#include "builtin_interfaces/msg/time.h"
// Member `stamp`
#include "builtin_interfaces/msg/detail/time__rosidl_typesupport_introspection_c.h"
// Member `instants`
#include "rj_msgs/msg/robot_instant.h"
// Member `instants`
#include "rj_msgs/msg/detail/robot_instant__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void Trajectory__rosidl_typesupport_introspection_c__Trajectory_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  rj_msgs__msg__Trajectory__init(message_memory);
}

void Trajectory__rosidl_typesupport_introspection_c__Trajectory_fini_function(void * message_memory)
{
  rj_msgs__msg__Trajectory__fini(message_memory);
}

size_t Trajectory__rosidl_typesupport_introspection_c__size_function__RobotInstant__instants(
  const void * untyped_member)
{
  const rj_msgs__msg__RobotInstant__Sequence * member =
    (const rj_msgs__msg__RobotInstant__Sequence *)(untyped_member);
  return member->size;
}

const void * Trajectory__rosidl_typesupport_introspection_c__get_const_function__RobotInstant__instants(
  const void * untyped_member, size_t index)
{
  const rj_msgs__msg__RobotInstant__Sequence * member =
    (const rj_msgs__msg__RobotInstant__Sequence *)(untyped_member);
  return &member->data[index];
}

void * Trajectory__rosidl_typesupport_introspection_c__get_function__RobotInstant__instants(
  void * untyped_member, size_t index)
{
  rj_msgs__msg__RobotInstant__Sequence * member =
    (rj_msgs__msg__RobotInstant__Sequence *)(untyped_member);
  return &member->data[index];
}

bool Trajectory__rosidl_typesupport_introspection_c__resize_function__RobotInstant__instants(
  void * untyped_member, size_t size)
{
  rj_msgs__msg__RobotInstant__Sequence * member =
    (rj_msgs__msg__RobotInstant__Sequence *)(untyped_member);
  rj_msgs__msg__RobotInstant__Sequence__fini(member);
  return rj_msgs__msg__RobotInstant__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember Trajectory__rosidl_typesupport_introspection_c__Trajectory_message_member_array[2] = {
  {
    "stamp",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rj_msgs__msg__Trajectory, stamp),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "instants",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rj_msgs__msg__Trajectory, instants),  // bytes offset in struct
    NULL,  // default value
    Trajectory__rosidl_typesupport_introspection_c__size_function__RobotInstant__instants,  // size() function pointer
    Trajectory__rosidl_typesupport_introspection_c__get_const_function__RobotInstant__instants,  // get_const(index) function pointer
    Trajectory__rosidl_typesupport_introspection_c__get_function__RobotInstant__instants,  // get(index) function pointer
    Trajectory__rosidl_typesupport_introspection_c__resize_function__RobotInstant__instants  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers Trajectory__rosidl_typesupport_introspection_c__Trajectory_message_members = {
  "rj_msgs__msg",  // message namespace
  "Trajectory",  // message name
  2,  // number of fields
  sizeof(rj_msgs__msg__Trajectory),
  Trajectory__rosidl_typesupport_introspection_c__Trajectory_message_member_array,  // message members
  Trajectory__rosidl_typesupport_introspection_c__Trajectory_init_function,  // function to initialize message memory (memory has to be allocated)
  Trajectory__rosidl_typesupport_introspection_c__Trajectory_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t Trajectory__rosidl_typesupport_introspection_c__Trajectory_message_type_support_handle = {
  0,
  &Trajectory__rosidl_typesupport_introspection_c__Trajectory_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_rj_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rj_msgs, msg, Trajectory)() {
  Trajectory__rosidl_typesupport_introspection_c__Trajectory_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, builtin_interfaces, msg, Time)();
  Trajectory__rosidl_typesupport_introspection_c__Trajectory_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rj_msgs, msg, RobotInstant)();
  if (!Trajectory__rosidl_typesupport_introspection_c__Trajectory_message_type_support_handle.typesupport_identifier) {
    Trajectory__rosidl_typesupport_introspection_c__Trajectory_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &Trajectory__rosidl_typesupport_introspection_c__Trajectory_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
