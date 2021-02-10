// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from rj_msgs:msg/PivotMotionCommand.idl
// generated code does not contain a copyright notice
#include "rj_msgs/msg/detail/pivot_motion_command__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


// Include directives for member types
// Member `pivot_point`
// Member `pivot_target`
#include "rj_geometry_msgs/msg/detail/point__functions.h"

bool
rj_msgs__msg__PivotMotionCommand__init(rj_msgs__msg__PivotMotionCommand * msg)
{
  if (!msg) {
    return false;
  }
  // pivot_point
  if (!rj_geometry_msgs__msg__Point__init(&msg->pivot_point)) {
    rj_msgs__msg__PivotMotionCommand__fini(msg);
    return false;
  }
  // pivot_target
  if (!rj_geometry_msgs__msg__Point__init(&msg->pivot_target)) {
    rj_msgs__msg__PivotMotionCommand__fini(msg);
    return false;
  }
  return true;
}

void
rj_msgs__msg__PivotMotionCommand__fini(rj_msgs__msg__PivotMotionCommand * msg)
{
  if (!msg) {
    return;
  }
  // pivot_point
  rj_geometry_msgs__msg__Point__fini(&msg->pivot_point);
  // pivot_target
  rj_geometry_msgs__msg__Point__fini(&msg->pivot_target);
}

rj_msgs__msg__PivotMotionCommand *
rj_msgs__msg__PivotMotionCommand__create()
{
  rj_msgs__msg__PivotMotionCommand * msg = (rj_msgs__msg__PivotMotionCommand *)malloc(sizeof(rj_msgs__msg__PivotMotionCommand));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(rj_msgs__msg__PivotMotionCommand));
  bool success = rj_msgs__msg__PivotMotionCommand__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
rj_msgs__msg__PivotMotionCommand__destroy(rj_msgs__msg__PivotMotionCommand * msg)
{
  if (msg) {
    rj_msgs__msg__PivotMotionCommand__fini(msg);
  }
  free(msg);
}


bool
rj_msgs__msg__PivotMotionCommand__Sequence__init(rj_msgs__msg__PivotMotionCommand__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rj_msgs__msg__PivotMotionCommand * data = NULL;
  if (size) {
    data = (rj_msgs__msg__PivotMotionCommand *)calloc(size, sizeof(rj_msgs__msg__PivotMotionCommand));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = rj_msgs__msg__PivotMotionCommand__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        rj_msgs__msg__PivotMotionCommand__fini(&data[i - 1]);
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
rj_msgs__msg__PivotMotionCommand__Sequence__fini(rj_msgs__msg__PivotMotionCommand__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      rj_msgs__msg__PivotMotionCommand__fini(&array->data[i]);
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

rj_msgs__msg__PivotMotionCommand__Sequence *
rj_msgs__msg__PivotMotionCommand__Sequence__create(size_t size)
{
  rj_msgs__msg__PivotMotionCommand__Sequence * array = (rj_msgs__msg__PivotMotionCommand__Sequence *)malloc(sizeof(rj_msgs__msg__PivotMotionCommand__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = rj_msgs__msg__PivotMotionCommand__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
rj_msgs__msg__PivotMotionCommand__Sequence__destroy(rj_msgs__msg__PivotMotionCommand__Sequence * array)
{
  if (array) {
    rj_msgs__msg__PivotMotionCommand__Sequence__fini(array);
  }
  free(array);
}
