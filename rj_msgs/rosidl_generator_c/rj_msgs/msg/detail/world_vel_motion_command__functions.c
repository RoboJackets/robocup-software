// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from rj_msgs:msg/WorldVelMotionCommand.idl
// generated code does not contain a copyright notice
#include "rj_msgs/msg/detail/world_vel_motion_command__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


// Include directives for member types
// Member `world_vel`
#include "rj_geometry_msgs/msg/detail/point__functions.h"

bool
rj_msgs__msg__WorldVelMotionCommand__init(rj_msgs__msg__WorldVelMotionCommand * msg)
{
  if (!msg) {
    return false;
  }
  // world_vel
  if (!rj_geometry_msgs__msg__Point__init(&msg->world_vel)) {
    rj_msgs__msg__WorldVelMotionCommand__fini(msg);
    return false;
  }
  return true;
}

void
rj_msgs__msg__WorldVelMotionCommand__fini(rj_msgs__msg__WorldVelMotionCommand * msg)
{
  if (!msg) {
    return;
  }
  // world_vel
  rj_geometry_msgs__msg__Point__fini(&msg->world_vel);
}

rj_msgs__msg__WorldVelMotionCommand *
rj_msgs__msg__WorldVelMotionCommand__create()
{
  rj_msgs__msg__WorldVelMotionCommand * msg = (rj_msgs__msg__WorldVelMotionCommand *)malloc(sizeof(rj_msgs__msg__WorldVelMotionCommand));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(rj_msgs__msg__WorldVelMotionCommand));
  bool success = rj_msgs__msg__WorldVelMotionCommand__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
rj_msgs__msg__WorldVelMotionCommand__destroy(rj_msgs__msg__WorldVelMotionCommand * msg)
{
  if (msg) {
    rj_msgs__msg__WorldVelMotionCommand__fini(msg);
  }
  free(msg);
}


bool
rj_msgs__msg__WorldVelMotionCommand__Sequence__init(rj_msgs__msg__WorldVelMotionCommand__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rj_msgs__msg__WorldVelMotionCommand * data = NULL;
  if (size) {
    data = (rj_msgs__msg__WorldVelMotionCommand *)calloc(size, sizeof(rj_msgs__msg__WorldVelMotionCommand));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = rj_msgs__msg__WorldVelMotionCommand__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        rj_msgs__msg__WorldVelMotionCommand__fini(&data[i - 1]);
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
rj_msgs__msg__WorldVelMotionCommand__Sequence__fini(rj_msgs__msg__WorldVelMotionCommand__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      rj_msgs__msg__WorldVelMotionCommand__fini(&array->data[i]);
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

rj_msgs__msg__WorldVelMotionCommand__Sequence *
rj_msgs__msg__WorldVelMotionCommand__Sequence__create(size_t size)
{
  rj_msgs__msg__WorldVelMotionCommand__Sequence * array = (rj_msgs__msg__WorldVelMotionCommand__Sequence *)malloc(sizeof(rj_msgs__msg__WorldVelMotionCommand__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = rj_msgs__msg__WorldVelMotionCommand__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
rj_msgs__msg__WorldVelMotionCommand__Sequence__destroy(rj_msgs__msg__WorldVelMotionCommand__Sequence * array)
{
  if (array) {
    rj_msgs__msg__WorldVelMotionCommand__Sequence__fini(array);
  }
  free(array);
}
