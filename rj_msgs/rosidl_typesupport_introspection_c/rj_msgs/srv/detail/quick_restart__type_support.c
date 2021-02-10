// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from rj_msgs:srv/QuickRestart.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "rj_msgs/srv/detail/quick_restart__rosidl_typesupport_introspection_c.h"
#include "rj_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "rj_msgs/srv/detail/quick_restart__functions.h"
#include "rj_msgs/srv/detail/quick_restart__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void QuickRestart_Request__rosidl_typesupport_introspection_c__QuickRestart_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  rj_msgs__srv__QuickRestart_Request__init(message_memory);
}

void QuickRestart_Request__rosidl_typesupport_introspection_c__QuickRestart_Request_fini_function(void * message_memory)
{
  rj_msgs__srv__QuickRestart_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember QuickRestart_Request__rosidl_typesupport_introspection_c__QuickRestart_Request_message_member_array[2] = {
  {
    "restart",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rj_msgs__srv__QuickRestart_Request, restart),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "blue_team",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rj_msgs__srv__QuickRestart_Request, blue_team),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers QuickRestart_Request__rosidl_typesupport_introspection_c__QuickRestart_Request_message_members = {
  "rj_msgs__srv",  // message namespace
  "QuickRestart_Request",  // message name
  2,  // number of fields
  sizeof(rj_msgs__srv__QuickRestart_Request),
  QuickRestart_Request__rosidl_typesupport_introspection_c__QuickRestart_Request_message_member_array,  // message members
  QuickRestart_Request__rosidl_typesupport_introspection_c__QuickRestart_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  QuickRestart_Request__rosidl_typesupport_introspection_c__QuickRestart_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t QuickRestart_Request__rosidl_typesupport_introspection_c__QuickRestart_Request_message_type_support_handle = {
  0,
  &QuickRestart_Request__rosidl_typesupport_introspection_c__QuickRestart_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_rj_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rj_msgs, srv, QuickRestart_Request)() {
  if (!QuickRestart_Request__rosidl_typesupport_introspection_c__QuickRestart_Request_message_type_support_handle.typesupport_identifier) {
    QuickRestart_Request__rosidl_typesupport_introspection_c__QuickRestart_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &QuickRestart_Request__rosidl_typesupport_introspection_c__QuickRestart_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "rj_msgs/srv/detail/quick_restart__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rj_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "rj_msgs/srv/detail/quick_restart__functions.h"
// already included above
// #include "rj_msgs/srv/detail/quick_restart__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void QuickRestart_Response__rosidl_typesupport_introspection_c__QuickRestart_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  rj_msgs__srv__QuickRestart_Response__init(message_memory);
}

void QuickRestart_Response__rosidl_typesupport_introspection_c__QuickRestart_Response_fini_function(void * message_memory)
{
  rj_msgs__srv__QuickRestart_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember QuickRestart_Response__rosidl_typesupport_introspection_c__QuickRestart_Response_message_member_array[1] = {
  {
    "structure_needs_at_least_one_member",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rj_msgs__srv__QuickRestart_Response, structure_needs_at_least_one_member),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers QuickRestart_Response__rosidl_typesupport_introspection_c__QuickRestart_Response_message_members = {
  "rj_msgs__srv",  // message namespace
  "QuickRestart_Response",  // message name
  1,  // number of fields
  sizeof(rj_msgs__srv__QuickRestart_Response),
  QuickRestart_Response__rosidl_typesupport_introspection_c__QuickRestart_Response_message_member_array,  // message members
  QuickRestart_Response__rosidl_typesupport_introspection_c__QuickRestart_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  QuickRestart_Response__rosidl_typesupport_introspection_c__QuickRestart_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t QuickRestart_Response__rosidl_typesupport_introspection_c__QuickRestart_Response_message_type_support_handle = {
  0,
  &QuickRestart_Response__rosidl_typesupport_introspection_c__QuickRestart_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_rj_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rj_msgs, srv, QuickRestart_Response)() {
  if (!QuickRestart_Response__rosidl_typesupport_introspection_c__QuickRestart_Response_message_type_support_handle.typesupport_identifier) {
    QuickRestart_Response__rosidl_typesupport_introspection_c__QuickRestart_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &QuickRestart_Response__rosidl_typesupport_introspection_c__QuickRestart_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "rj_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rj_msgs/srv/detail/quick_restart__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers rj_msgs__srv__detail__quick_restart__rosidl_typesupport_introspection_c__QuickRestart_service_members = {
  "rj_msgs__srv",  // service namespace
  "QuickRestart",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // rj_msgs__srv__detail__quick_restart__rosidl_typesupport_introspection_c__QuickRestart_Request_message_type_support_handle,
  NULL  // response message
  // rj_msgs__srv__detail__quick_restart__rosidl_typesupport_introspection_c__QuickRestart_Response_message_type_support_handle
};

static rosidl_service_type_support_t rj_msgs__srv__detail__quick_restart__rosidl_typesupport_introspection_c__QuickRestart_service_type_support_handle = {
  0,
  &rj_msgs__srv__detail__quick_restart__rosidl_typesupport_introspection_c__QuickRestart_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rj_msgs, srv, QuickRestart_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rj_msgs, srv, QuickRestart_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_rj_msgs
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rj_msgs, srv, QuickRestart)() {
  if (!rj_msgs__srv__detail__quick_restart__rosidl_typesupport_introspection_c__QuickRestart_service_type_support_handle.typesupport_identifier) {
    rj_msgs__srv__detail__quick_restart__rosidl_typesupport_introspection_c__QuickRestart_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)rj_msgs__srv__detail__quick_restart__rosidl_typesupport_introspection_c__QuickRestart_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rj_msgs, srv, QuickRestart_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rj_msgs, srv, QuickRestart_Response)()->data;
  }

  return &rj_msgs__srv__detail__quick_restart__rosidl_typesupport_introspection_c__QuickRestart_service_type_support_handle;
}
