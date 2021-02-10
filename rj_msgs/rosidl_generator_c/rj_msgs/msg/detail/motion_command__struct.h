// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rj_msgs:msg/MotionCommand.idl
// generated code does not contain a copyright notice

#ifndef RJ_MSGS__MSG__DETAIL__MOTION_COMMAND__STRUCT_H_
#define RJ_MSGS__MSG__DETAIL__MOTION_COMMAND__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'empty_command'
#include "rj_msgs/msg/detail/empty_motion_command__struct.h"
// Member 'path_target_command'
#include "rj_msgs/msg/detail/path_target_motion_command__struct.h"
// Member 'world_vel_command'
#include "rj_msgs/msg/detail/world_vel_motion_command__struct.h"
// Member 'pivot_command'
#include "rj_msgs/msg/detail/pivot_motion_command__struct.h"
// Member 'settle_command'
#include "rj_msgs/msg/detail/settle_motion_command__struct.h"
// Member 'collect_command'
#include "rj_msgs/msg/detail/collect_motion_command__struct.h"
// Member 'line_kick_command'
#include "rj_msgs/msg/detail/line_kick_motion_command__struct.h"
// Member 'intercept_command'
#include "rj_msgs/msg/detail/intercept_motion_command__struct.h"

// constants for array fields with an upper bound
// empty_command
enum
{
  rj_msgs__msg__MotionCommand__empty_command__MAX_SIZE = 1
};
// path_target_command
enum
{
  rj_msgs__msg__MotionCommand__path_target_command__MAX_SIZE = 1
};
// world_vel_command
enum
{
  rj_msgs__msg__MotionCommand__world_vel_command__MAX_SIZE = 1
};
// pivot_command
enum
{
  rj_msgs__msg__MotionCommand__pivot_command__MAX_SIZE = 1
};
// settle_command
enum
{
  rj_msgs__msg__MotionCommand__settle_command__MAX_SIZE = 1
};
// collect_command
enum
{
  rj_msgs__msg__MotionCommand__collect_command__MAX_SIZE = 1
};
// line_kick_command
enum
{
  rj_msgs__msg__MotionCommand__line_kick_command__MAX_SIZE = 1
};
// intercept_command
enum
{
  rj_msgs__msg__MotionCommand__intercept_command__MAX_SIZE = 1
};

// Struct defined in msg/MotionCommand in the package rj_msgs.
typedef struct rj_msgs__msg__MotionCommand
{
  rj_msgs__msg__EmptyMotionCommand__Sequence empty_command;
  rj_msgs__msg__PathTargetMotionCommand__Sequence path_target_command;
  rj_msgs__msg__WorldVelMotionCommand__Sequence world_vel_command;
  rj_msgs__msg__PivotMotionCommand__Sequence pivot_command;
  rj_msgs__msg__SettleMotionCommand__Sequence settle_command;
  rj_msgs__msg__CollectMotionCommand__Sequence collect_command;
  rj_msgs__msg__LineKickMotionCommand__Sequence line_kick_command;
  rj_msgs__msg__InterceptMotionCommand__Sequence intercept_command;
} rj_msgs__msg__MotionCommand;

// Struct for a sequence of rj_msgs__msg__MotionCommand.
typedef struct rj_msgs__msg__MotionCommand__Sequence
{
  rj_msgs__msg__MotionCommand * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rj_msgs__msg__MotionCommand__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // RJ_MSGS__MSG__DETAIL__MOTION_COMMAND__STRUCT_H_
