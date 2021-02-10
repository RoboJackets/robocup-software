// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rj_msgs:msg/RobotStatus.idl
// generated code does not contain a copyright notice

#ifndef RJ_MSGS__MSG__DETAIL__ROBOT_STATUS__STRUCT_H_
#define RJ_MSGS__MSG__DETAIL__ROBOT_STATUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in msg/RobotStatus in the package rj_msgs.
typedef struct rj_msgs__msg__RobotStatus
{
  uint8_t robot_id;
  double battery_voltage;
  bool motor_errors[5];
  bool has_ball_sense;
  bool kicker_charged;
  bool kicker_healthy;
  bool fpga_error;
  int16_t encoder_deltas[4];
} rj_msgs__msg__RobotStatus;

// Struct for a sequence of rj_msgs__msg__RobotStatus.
typedef struct rj_msgs__msg__RobotStatus__Sequence
{
  rj_msgs__msg__RobotStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rj_msgs__msg__RobotStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // RJ_MSGS__MSG__DETAIL__ROBOT_STATUS__STRUCT_H_
