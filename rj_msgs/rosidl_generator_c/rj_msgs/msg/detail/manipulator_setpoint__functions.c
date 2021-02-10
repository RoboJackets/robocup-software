// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from rj_msgs:msg/ManipulatorSetpoint.idl
// generated code does not contain a copyright notice
#include "rj_msgs/msg/detail/manipulator_setpoint__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


bool
rj_msgs__msg__ManipulatorSetpoint__init(rj_msgs__msg__ManipulatorSetpoint * msg)
{
  if (!msg) {
    return false;
  }
  // shoot_mode
  // trigger_mode
  // kick_strength
  // dribbler_speed
  return true;
}

void
rj_msgs__msg__ManipulatorSetpoint__fini(rj_msgs__msg__ManipulatorSetpoint * msg)
{
  if (!msg) {
    return;
  }
  // shoot_mode
  // trigger_mode
  // kick_strength
  // dribbler_speed
}

rj_msgs__msg__ManipulatorSetpoint *
rj_msgs__msg__ManipulatorSetpoint__create()
{
  rj_msgs__msg__ManipulatorSetpoint * msg = (rj_msgs__msg__ManipulatorSetpoint *)malloc(sizeof(rj_msgs__msg__ManipulatorSetpoint));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(rj_msgs__msg__ManipulatorSetpoint));
  bool success = rj_msgs__msg__ManipulatorSetpoint__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
rj_msgs__msg__ManipulatorSetpoint__destroy(rj_msgs__msg__ManipulatorSetpoint * msg)
{
  if (msg) {
    rj_msgs__msg__ManipulatorSetpoint__fini(msg);
  }
  free(msg);
}


bool
rj_msgs__msg__ManipulatorSetpoint__Sequence__init(rj_msgs__msg__ManipulatorSetpoint__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rj_msgs__msg__ManipulatorSetpoint * data = NULL;
  if (size) {
    data = (rj_msgs__msg__ManipulatorSetpoint *)calloc(size, sizeof(rj_msgs__msg__ManipulatorSetpoint));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = rj_msgs__msg__ManipulatorSetpoint__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        rj_msgs__msg__ManipulatorSetpoint__fini(&data[i - 1]);
      }
      free(data);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
rj_msgs__msg__ManipulatorSetpoint__Sequence__fini(rj_msgs__msg__ManipulatorSetpoint__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      rj_msgs__msg__ManipulatorSetpoint__fini(&array->data[i]);
    }
    free(array->data);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

rj_msgs__msg__ManipulatorSetpoint__Sequence *
rj_msgs__msg__ManipulatorSetpoint__Sequence__create(size_t size)
{
  rj_msgs__msg__ManipulatorSetpoint__Sequence * array = (rj_msgs__msg__ManipulatorSetpoint__Sequence *)malloc(sizeof(rj_msgs__msg__ManipulatorSetpoint__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = rj_msgs__msg__ManipulatorSetpoint__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
rj_msgs__msg__ManipulatorSetpoint__Sequence__destroy(rj_msgs__msg__ManipulatorSetpoint__Sequence * array)
{
  if (array) {
    rj_msgs__msg__ManipulatorSetpoint__Sequence__fini(array);
  }
  free(array);
}
