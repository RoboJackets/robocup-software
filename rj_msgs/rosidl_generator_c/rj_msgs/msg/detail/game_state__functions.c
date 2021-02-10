// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from rj_msgs:msg/GameState.idl
// generated code does not contain a copyright notice
#include "rj_msgs/msg/detail/game_state__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


// Include directives for member types
// Member `stage_time_left`
#include "builtin_interfaces/msg/detail/duration__functions.h"
// Member `placement_point`
#include "rj_geometry_msgs/msg/detail/point__functions.h"

bool
rj_msgs__msg__GameState__init(rj_msgs__msg__GameState * msg)
{
  if (!msg) {
    return false;
  }
  // period
  // state
  // restart
  // our_restart
  // stage_time_left
  if (!builtin_interfaces__msg__Duration__init(&msg->stage_time_left)) {
    rj_msgs__msg__GameState__fini(msg);
    return false;
  }
  // placement_point
  if (!rj_geometry_msgs__msg__Point__init(&msg->placement_point)) {
    rj_msgs__msg__GameState__fini(msg);
    return false;
  }
  return true;
}

void
rj_msgs__msg__GameState__fini(rj_msgs__msg__GameState * msg)
{
  if (!msg) {
    return;
  }
  // period
  // state
  // restart
  // our_restart
  // stage_time_left
  builtin_interfaces__msg__Duration__fini(&msg->stage_time_left);
  // placement_point
  rj_geometry_msgs__msg__Point__fini(&msg->placement_point);
}

rj_msgs__msg__GameState *
rj_msgs__msg__GameState__create()
{
  rj_msgs__msg__GameState * msg = (rj_msgs__msg__GameState *)malloc(sizeof(rj_msgs__msg__GameState));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(rj_msgs__msg__GameState));
  bool success = rj_msgs__msg__GameState__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
rj_msgs__msg__GameState__destroy(rj_msgs__msg__GameState * msg)
{
  if (msg) {
    rj_msgs__msg__GameState__fini(msg);
  }
  free(msg);
}


bool
rj_msgs__msg__GameState__Sequence__init(rj_msgs__msg__GameState__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rj_msgs__msg__GameState * data = NULL;
  if (size) {
    data = (rj_msgs__msg__GameState *)calloc(size, sizeof(rj_msgs__msg__GameState));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = rj_msgs__msg__GameState__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        rj_msgs__msg__GameState__fini(&data[i - 1]);
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
rj_msgs__msg__GameState__Sequence__fini(rj_msgs__msg__GameState__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      rj_msgs__msg__GameState__fini(&array->data[i]);
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

rj_msgs__msg__GameState__Sequence *
rj_msgs__msg__GameState__Sequence__create(size_t size)
{
  rj_msgs__msg__GameState__Sequence * array = (rj_msgs__msg__GameState__Sequence *)malloc(sizeof(rj_msgs__msg__GameState__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = rj_msgs__msg__GameState__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
rj_msgs__msg__GameState__Sequence__destroy(rj_msgs__msg__GameState__Sequence * array)
{
  if (array) {
    rj_msgs__msg__GameState__Sequence__fini(array);
  }
  free(array);
}
