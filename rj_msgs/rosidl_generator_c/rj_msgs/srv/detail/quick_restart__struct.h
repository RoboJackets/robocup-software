// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rj_msgs:srv/QuickRestart.idl
// generated code does not contain a copyright notice

#ifndef RJ_MSGS__SRV__DETAIL__QUICK_RESTART__STRUCT_H_
#define RJ_MSGS__SRV__DETAIL__QUICK_RESTART__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'RESTART_KICKOFF'.
enum
{
  rj_msgs__srv__QuickRestart_Request__RESTART_KICKOFF = 0
};

/// Constant 'RESTART_DIRECT'.
enum
{
  rj_msgs__srv__QuickRestart_Request__RESTART_DIRECT = 1
};

/// Constant 'RESTART_INDIRECT'.
enum
{
  rj_msgs__srv__QuickRestart_Request__RESTART_INDIRECT = 2
};

// Struct defined in srv/QuickRestart in the package rj_msgs.
typedef struct rj_msgs__srv__QuickRestart_Request
{
  uint8_t restart;
  bool blue_team;
} rj_msgs__srv__QuickRestart_Request;

// Struct for a sequence of rj_msgs__srv__QuickRestart_Request.
typedef struct rj_msgs__srv__QuickRestart_Request__Sequence
{
  rj_msgs__srv__QuickRestart_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rj_msgs__srv__QuickRestart_Request__Sequence;


// Constants defined in the message

// Struct defined in srv/QuickRestart in the package rj_msgs.
typedef struct rj_msgs__srv__QuickRestart_Response
{
  uint8_t structure_needs_at_least_one_member;
} rj_msgs__srv__QuickRestart_Response;

// Struct for a sequence of rj_msgs__srv__QuickRestart_Response.
typedef struct rj_msgs__srv__QuickRestart_Response__Sequence
{
  rj_msgs__srv__QuickRestart_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rj_msgs__srv__QuickRestart_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // RJ_MSGS__SRV__DETAIL__QUICK_RESTART__STRUCT_H_
