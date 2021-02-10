// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rj_msgs:msg/RobotIntent.idl
// generated code does not contain a copyright notice

#ifndef RJ_MSGS__MSG__DETAIL__ROBOT_INTENT__STRUCT_H_
#define RJ_MSGS__MSG__DETAIL__ROBOT_INTENT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'SHOOT_MODE_KICK'.
enum
{
  rj_msgs__msg__RobotIntent__SHOOT_MODE_KICK = 0
};

/// Constant 'SHOOT_MODE_CHIP'.
enum
{
  rj_msgs__msg__RobotIntent__SHOOT_MODE_CHIP = 1
};

/// Constant 'TRIGGER_MODE_STAND_DOWN'.
enum
{
  rj_msgs__msg__RobotIntent__TRIGGER_MODE_STAND_DOWN = 0
};

/// Constant 'TRIGGER_MODE_IMMEDIATE'.
enum
{
  rj_msgs__msg__RobotIntent__TRIGGER_MODE_IMMEDIATE = 1
};

/// Constant 'TRIGGER_MODE_ON_BREAK_BEAM'.
enum
{
  rj_msgs__msg__RobotIntent__TRIGGER_MODE_ON_BREAK_BEAM = 2
};

// Include directives for member types
// Member 'motion_command'
#include "rj_msgs/msg/detail/motion_command__struct.h"
// Member 'local_obstacles'
#include "rj_geometry_msgs/msg/detail/shape_set__struct.h"

// Struct defined in msg/RobotIntent in the package rj_msgs.
typedef struct rj_msgs__msg__RobotIntent
{
  rj_msgs__msg__MotionCommand motion_command;
  rj_geometry_msgs__msg__ShapeSet local_obstacles;
  uint8_t shoot_mode;
  uint8_t trigger_mode;
  float kick_speed;
  float dribbler_speed;
  bool is_active;
  int8_t priority;
} rj_msgs__msg__RobotIntent;

// Struct for a sequence of rj_msgs__msg__RobotIntent.
typedef struct rj_msgs__msg__RobotIntent__Sequence
{
  rj_msgs__msg__RobotIntent * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rj_msgs__msg__RobotIntent__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // RJ_MSGS__MSG__DETAIL__ROBOT_INTENT__STRUCT_H_
