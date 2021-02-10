// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from rj_msgs:srv/QuickRestart.idl
// generated code does not contain a copyright notice
#include "rj_msgs/srv/detail/quick_restart__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rj_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "rj_msgs/srv/detail/quick_restart__struct.h"
#include "rj_msgs/srv/detail/quick_restart__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif


// forward declare type support functions


using _QuickRestart_Request__ros_msg_type = rj_msgs__srv__QuickRestart_Request;

static bool _QuickRestart_Request__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _QuickRestart_Request__ros_msg_type * ros_message = static_cast<const _QuickRestart_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: restart
  {
    cdr << ros_message->restart;
  }

  // Field name: blue_team
  {
    cdr << (ros_message->blue_team ? true : false);
  }

  return true;
}

static bool _QuickRestart_Request__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _QuickRestart_Request__ros_msg_type * ros_message = static_cast<_QuickRestart_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: restart
  {
    cdr >> ros_message->restart;
  }

  // Field name: blue_team
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->blue_team = tmp ? true : false;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_rj_msgs
size_t get_serialized_size_rj_msgs__srv__QuickRestart_Request(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _QuickRestart_Request__ros_msg_type * ros_message = static_cast<const _QuickRestart_Request__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name restart
  {
    size_t item_size = sizeof(ros_message->restart);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name blue_team
  {
    size_t item_size = sizeof(ros_message->blue_team);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _QuickRestart_Request__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_rj_msgs__srv__QuickRestart_Request(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_rj_msgs
size_t max_serialized_size_rj_msgs__srv__QuickRestart_Request(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;

  // member: restart
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: blue_team
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  return current_alignment - initial_alignment;
}

static size_t _QuickRestart_Request__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_rj_msgs__srv__QuickRestart_Request(
    full_bounded, 0);
}


static message_type_support_callbacks_t __callbacks_QuickRestart_Request = {
  "rj_msgs::srv",
  "QuickRestart_Request",
  _QuickRestart_Request__cdr_serialize,
  _QuickRestart_Request__cdr_deserialize,
  _QuickRestart_Request__get_serialized_size,
  _QuickRestart_Request__max_serialized_size
};

static rosidl_message_type_support_t _QuickRestart_Request__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_QuickRestart_Request,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, rj_msgs, srv, QuickRestart_Request)() {
  return &_QuickRestart_Request__type_support;
}

#if defined(__cplusplus)
}
#endif

// already included above
// #include <cassert>
// already included above
// #include <limits>
// already included above
// #include <string>
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
// already included above
// #include "rj_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
// already included above
// #include "rj_msgs/srv/detail/quick_restart__struct.h"
// already included above
// #include "rj_msgs/srv/detail/quick_restart__functions.h"
// already included above
// #include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif


// forward declare type support functions


using _QuickRestart_Response__ros_msg_type = rj_msgs__srv__QuickRestart_Response;

static bool _QuickRestart_Response__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _QuickRestart_Response__ros_msg_type * ros_message = static_cast<const _QuickRestart_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: structure_needs_at_least_one_member
  {
    cdr << ros_message->structure_needs_at_least_one_member;
  }

  return true;
}

static bool _QuickRestart_Response__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _QuickRestart_Response__ros_msg_type * ros_message = static_cast<_QuickRestart_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: structure_needs_at_least_one_member
  {
    cdr >> ros_message->structure_needs_at_least_one_member;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_rj_msgs
size_t get_serialized_size_rj_msgs__srv__QuickRestart_Response(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _QuickRestart_Response__ros_msg_type * ros_message = static_cast<const _QuickRestart_Response__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name structure_needs_at_least_one_member
  {
    size_t item_size = sizeof(ros_message->structure_needs_at_least_one_member);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _QuickRestart_Response__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_rj_msgs__srv__QuickRestart_Response(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_rj_msgs
size_t max_serialized_size_rj_msgs__srv__QuickRestart_Response(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;

  // member: structure_needs_at_least_one_member
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  return current_alignment - initial_alignment;
}

static size_t _QuickRestart_Response__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_rj_msgs__srv__QuickRestart_Response(
    full_bounded, 0);
}


static message_type_support_callbacks_t __callbacks_QuickRestart_Response = {
  "rj_msgs::srv",
  "QuickRestart_Response",
  _QuickRestart_Response__cdr_serialize,
  _QuickRestart_Response__cdr_deserialize,
  _QuickRestart_Response__get_serialized_size,
  _QuickRestart_Response__max_serialized_size
};

static rosidl_message_type_support_t _QuickRestart_Response__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_QuickRestart_Response,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, rj_msgs, srv, QuickRestart_Response)() {
  return &_QuickRestart_Response__type_support;
}

#if defined(__cplusplus)
}
#endif

#include "rosidl_typesupport_fastrtps_cpp/service_type_support.h"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "rj_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "rj_msgs/srv/quick_restart.h"

#if defined(__cplusplus)
extern "C"
{
#endif

static service_type_support_callbacks_t QuickRestart__callbacks = {
  "rj_msgs::srv",
  "QuickRestart",
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, rj_msgs, srv, QuickRestart_Request)(),
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, rj_msgs, srv, QuickRestart_Response)(),
};

static rosidl_service_type_support_t QuickRestart__handle = {
  rosidl_typesupport_fastrtps_c__identifier,
  &QuickRestart__callbacks,
  get_service_typesupport_handle_function,
};

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, rj_msgs, srv, QuickRestart)() {
  return &QuickRestart__handle;
}

#if defined(__cplusplus)
}
#endif
