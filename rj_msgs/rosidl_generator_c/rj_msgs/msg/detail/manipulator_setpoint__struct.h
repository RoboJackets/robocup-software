// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rj_msgs:msg/ManipulatorSetpoint.idl
// generated code does not contain a copyright notice

#ifndef RJ_MSGS__MSG__DETAIL__MANIPULATOR_SETPOINT__STRUCT_H_
#define RJ_MSGS__MSG__DETAIL__MANIPULATOR_SETPOINT__STRUCT_H_

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
  rj_msgs__msg__ManipulatorSetpoint__SHOOT_MODE_KICK = 0
};

/// Constant 'SHOOT_MODE_CHIP'.
enum
{
  rj_msgs__msg__ManipulatorSetpoint__SHOOT_MODE_CHIP = 1
};

/// Constant 'TRIGGER_MODE_STAND_DOWN'.
enum
{
  rj_msgs__msg__ManipulatorSetpoint__TRIGGER_MODE_STAND_DOWN = 0
};

/// Constant 'TRIGGER_MODE_IMMEDIATE'.
enum
{
  rj_msgs__msg__ManipulatorSetpoint__TRIGGER_MODE_IMMEDIATE = 1
};

/// Constant 'TRIGGER_MODE_ON_BREAK_BEAM'.
enum
{
  rj_msgs__msg__ManipulatorSetpoint__TRIGGER_MODE_ON_BREAK_BEAM = 2
};

// Struct defined in msg/ManipulatorSetpoint in the package rj_msgs.
typedef struct rj_msgs__msg__ManipulatorSetpoint
{
  uint8_t shoot_mode;
  uint8_t trigger_mode;
  int8_t kick_strength;
  float dribbler_speed;
} rj_msgs__msg__ManipulatorSetpoint;

// Struct for a sequence of rj_msgs__msg__ManipulatorSetpoint.
typedef struct rj_msgs__msg__ManipulatorSetpoint__Sequence
{
  rj_msgs__msg__ManipulatorSetpoint * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rj_msgs__msg__ManipulatorSetpoint__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // RJ_MSGS__MSG__DETAIL__MANIPULATOR_SETPOINT__STRUCT_H_
