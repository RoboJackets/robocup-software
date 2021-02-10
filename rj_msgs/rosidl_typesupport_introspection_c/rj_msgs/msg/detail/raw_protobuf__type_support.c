// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from rj_msgs:msg/RawProtobuf.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "rj_msgs/msg/detail/raw_protobuf__rosidl_typesupport_introspection_c.h"
#include "rj_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "rj_msgs/msg/detail/raw_protobuf__functions.h"
#include "rj_msgs/msg/detail/raw_protobuf__struct.h"


// Include directives for member types
// Member `data`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void RawProtobuf__rosidl_typesupport_introspection_c__RawProtobuf_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  rj_msgs__msg__RawProtobuf__init(message_memory);
}

void RawProtobuf__rosidl_typesupport_introspection_c__RawProtobuf_fini_function(void * message_memory)
{
  rj_msgs__msg__RawProtobuf__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember RawProtobuf__rosidl_typesupport_introspection_c__RawProtobuf_message_member_array[1] = {
  {
    "data",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_OCTET,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rj_msgs__msg__RawProtobuf, data),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers RawProtobuf__rosidl_typesupport_introspection_c__RawProtobuf_message_members = {
  "rj_msgs__msg",  // message namespace
  "RawProtobuf",  // message name
  1,  // number of fields
  sizeof(rj_msgs__msg__RawProtobuf),
  RawProtobuf__rosidl_typesupport_introspection_c__RawProtobuf_message_member_array,  // message members
  RawProtobuf__rosidl_typesupport_introspection_c__RawProtobuf_init_function,  // function to initialize message memory (memory has to be allocated)
  RawProtobuf__rosidl_typesupport_introspection_c__RawProtobuf_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t RawProtobuf__rosidl_typesupport_introspection_c__RawProtobuf_message_type_support_handle = {
  0,
  &RawProtobuf__rosidl_typesupport_introspection_c__RawProtobuf_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_rj_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rj_msgs, msg, RawProtobuf)() {
  if (!RawProtobuf__rosidl_typesupport_introspection_c__RawProtobuf_message_type_support_handle.typesupport_identifier) {
    RawProtobuf__rosidl_typesupport_introspection_c__RawProtobuf_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &RawProtobuf__rosidl_typesupport_introspection_c__RawProtobuf_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
