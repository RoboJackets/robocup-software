// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from rj_msgs:srv/SetGameSettings.idl
// generated code does not contain a copyright notice
#include "rj_msgs/srv/detail/set_game_settings__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rj_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "rj_msgs/srv/detail/set_game_settings__struct.h"
#include "rj_msgs/srv/detail/set_game_settings__functions.h"
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

#include "rj_msgs/msg/detail/game_settings__functions.h"  // game_settings

// forward declare type support functions
size_t get_serialized_size_rj_msgs__msg__GameSettings(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_rj_msgs__msg__GameSettings(
  bool & full_bounded,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, rj_msgs, msg, GameSettings)();


using _SetGameSettings_Request__ros_msg_type = rj_msgs__srv__SetGameSettings_Request;

static bool _SetGameSettings_Request__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _SetGameSettings_Request__ros_msg_type * ros_message = static_cast<const _SetGameSettings_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: game_settings
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, rj_msgs, msg, GameSettings
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->game_settings, cdr))
    {
      return false;
    }
  }

  return true;
}

static bool _SetGameSettings_Request__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _SetGameSettings_Request__ros_msg_type * ros_message = static_cast<_SetGameSettings_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: game_settings
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, rj_msgs, msg, GameSettings
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->game_settings))
    {
      return false;
    }
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_rj_msgs
size_t get_serialized_size_rj_msgs__srv__SetGameSettings_Request(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _SetGameSettings_Request__ros_msg_type * ros_message = static_cast<const _SetGameSettings_Request__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name game_settings

  current_alignment += get_serialized_size_rj_msgs__msg__GameSettings(
    &(ros_message->game_settings), current_alignment);

  return current_alignment - initial_alignment;
}

static uint32_t _SetGameSettings_Request__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_rj_msgs__srv__SetGameSettings_Request(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_rj_msgs
size_t max_serialized_size_rj_msgs__srv__SetGameSettings_Request(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;

  // member: game_settings
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        max_serialized_size_rj_msgs__msg__GameSettings(
        full_bounded, current_alignment);
    }
  }

  return current_alignment - initial_alignment;
}

static size_t _SetGameSettings_Request__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_rj_msgs__srv__SetGameSettings_Request(
    full_bounded, 0);
}


static message_type_support_callbacks_t __callbacks_SetGameSettings_Request = {
  "rj_msgs::srv",
  "SetGameSettings_Request",
  _SetGameSettings_Request__cdr_serialize,
  _SetGameSettings_Request__cdr_deserialize,
  _SetGameSettings_Request__get_serialized_size,
  _SetGameSettings_Request__max_serialized_size
};

static rosidl_message_type_support_t _SetGameSettings_Request__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_SetGameSettings_Request,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, rj_msgs, srv, SetGameSettings_Request)() {
  return &_SetGameSettings_Request__type_support;
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
// #include "rj_msgs/srv/detail/set_game_settings__struct.h"
// already included above
// #include "rj_msgs/srv/detail/set_game_settings__functions.h"
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


using _SetGameSettings_Response__ros_msg_type = rj_msgs__srv__SetGameSettings_Response;

static bool _SetGameSettings_Response__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _SetGameSettings_Response__ros_msg_type * ros_message = static_cast<const _SetGameSettings_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: structure_needs_at_least_one_member
  {
    cdr << ros_message->structure_needs_at_least_one_member;
  }

  return true;
}

static bool _SetGameSettings_Response__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _SetGameSettings_Response__ros_msg_type * ros_message = static_cast<_SetGameSettings_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: structure_needs_at_least_one_member
  {
    cdr >> ros_message->structure_needs_at_least_one_member;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_rj_msgs
size_t get_serialized_size_rj_msgs__srv__SetGameSettings_Response(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _SetGameSettings_Response__ros_msg_type * ros_message = static_cast<const _SetGameSettings_Response__ros_msg_type *>(untyped_ros_message);
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

static uint32_t _SetGameSettings_Response__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_rj_msgs__srv__SetGameSettings_Response(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_rj_msgs
size_t max_serialized_size_rj_msgs__srv__SetGameSettings_Response(
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

static size_t _SetGameSettings_Response__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_rj_msgs__srv__SetGameSettings_Response(
    full_bounded, 0);
}


static message_type_support_callbacks_t __callbacks_SetGameSettings_Response = {
  "rj_msgs::srv",
  "SetGameSettings_Response",
  _SetGameSettings_Response__cdr_serialize,
  _SetGameSettings_Response__cdr_deserialize,
  _SetGameSettings_Response__get_serialized_size,
  _SetGameSettings_Response__max_serialized_size
};

static rosidl_message_type_support_t _SetGameSettings_Response__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_SetGameSettings_Response,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, rj_msgs, srv, SetGameSettings_Response)() {
  return &_SetGameSettings_Response__type_support;
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
#include "rj_msgs/srv/set_game_settings.h"

#if defined(__cplusplus)
extern "C"
{
#endif

static service_type_support_callbacks_t SetGameSettings__callbacks = {
  "rj_msgs::srv",
  "SetGameSettings",
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, rj_msgs, srv, SetGameSettings_Request)(),
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, rj_msgs, srv, SetGameSettings_Response)(),
};

static rosidl_service_type_support_t SetGameSettings__handle = {
  rosidl_typesupport_fastrtps_c__identifier,
  &SetGameSettings__callbacks,
  get_service_typesupport_handle_function,
};

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, rj_msgs, srv, SetGameSettings)() {
  return &SetGameSettings__handle;
}

#if defined(__cplusplus)
}
#endif
