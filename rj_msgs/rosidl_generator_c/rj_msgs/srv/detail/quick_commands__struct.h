// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rj_msgs:srv/QuickCommands.idl
// generated code does not contain a copyright notice

#ifndef RJ_MSGS__SRV__DETAIL__QUICK_COMMANDS__STRUCT_H_
#define RJ_MSGS__SRV__DETAIL__QUICK_COMMANDS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'COMMAND_HALT'.
enum
{
  rj_msgs__srv__QuickCommands_Request__COMMAND_HALT = 0
};

/// Constant 'COMMAND_STOP'.
enum
{
  rj_msgs__srv__QuickCommands_Request__COMMAND_STOP = 1
};

/// Constant 'COMMAND_READY'.
enum
{
  rj_msgs__srv__QuickCommands_Request__COMMAND_READY = 2
};

/// Constant 'COMMAND_PLAY'.
enum
{
  rj_msgs__srv__QuickCommands_Request__COMMAND_PLAY = 3
};

// Struct defined in srv/QuickCommands in the package rj_msgs.
typedef struct rj_msgs__srv__QuickCommands_Request
{
  uint8_t state;
} rj_msgs__srv__QuickCommands_Request;

// Struct for a sequence of rj_msgs__srv__QuickCommands_Request.
typedef struct rj_msgs__srv__QuickCommands_Request__Sequence
{
  rj_msgs__srv__QuickCommands_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rj_msgs__srv__QuickCommands_Request__Sequence;


// Constants defined in the message

// Struct defined in srv/QuickCommands in the package rj_msgs.
typedef struct rj_msgs__srv__QuickCommands_Response
{
  uint8_t structure_needs_at_least_one_member;
} rj_msgs__srv__QuickCommands_Response;

// Struct for a sequence of rj_msgs__srv__QuickCommands_Response.
typedef struct rj_msgs__srv__QuickCommands_Response__Sequence
{
  rj_msgs__srv__QuickCommands_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rj_msgs__srv__QuickCommands_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // RJ_MSGS__SRV__DETAIL__QUICK_COMMANDS__STRUCT_H_
