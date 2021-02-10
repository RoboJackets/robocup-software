// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rj_msgs:srv/SetGameSettings.idl
// generated code does not contain a copyright notice

#ifndef RJ_MSGS__SRV__DETAIL__SET_GAME_SETTINGS__STRUCT_H_
#define RJ_MSGS__SRV__DETAIL__SET_GAME_SETTINGS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'game_settings'
#include "rj_msgs/msg/detail/game_settings__struct.h"

// Struct defined in srv/SetGameSettings in the package rj_msgs.
typedef struct rj_msgs__srv__SetGameSettings_Request
{
  rj_msgs__msg__GameSettings game_settings;
} rj_msgs__srv__SetGameSettings_Request;

// Struct for a sequence of rj_msgs__srv__SetGameSettings_Request.
typedef struct rj_msgs__srv__SetGameSettings_Request__Sequence
{
  rj_msgs__srv__SetGameSettings_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rj_msgs__srv__SetGameSettings_Request__Sequence;


// Constants defined in the message

// Struct defined in srv/SetGameSettings in the package rj_msgs.
typedef struct rj_msgs__srv__SetGameSettings_Response
{
  uint8_t structure_needs_at_least_one_member;
} rj_msgs__srv__SetGameSettings_Response;

// Struct for a sequence of rj_msgs__srv__SetGameSettings_Response.
typedef struct rj_msgs__srv__SetGameSettings_Response__Sequence
{
  rj_msgs__srv__SetGameSettings_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rj_msgs__srv__SetGameSettings_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // RJ_MSGS__SRV__DETAIL__SET_GAME_SETTINGS__STRUCT_H_
