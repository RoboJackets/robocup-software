// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from rj_msgs:msg/Trajectory.idl
// generated code does not contain a copyright notice
#include "rj_msgs/msg/detail/trajectory__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


// Include directives for member types
// Member `stamp`
#include "builtin_interfaces/msg/detail/time__functions.h"
// Member `instants`
#include "rj_msgs/msg/detail/robot_instant__functions.h"

bool
rj_msgs__msg__Trajectory__init(rj_msgs__msg__Trajectory * msg)
{
  if (!msg) {
    return false;
  }
  // stamp
  if (!builtin_interfaces__msg__Time__init(&msg->stamp)) {
    rj_msgs__msg__Trajectory__fini(msg);
    return false;
  }
  // instants
  if (!rj_msgs__msg__RobotInstant__Sequence__init(&msg->instants, 0)) {
    rj_msgs__msg__Trajectory__fini(msg);
    return false;
  }
  return true;
}

void
rj_msgs__msg__Trajectory__fini(rj_msgs__msg__Trajectory * msg)
{
  if (!msg) {
    return;
  }
  // stamp
  builtin_interfaces__msg__Time__fini(&msg->stamp);
  // instants
  rj_msgs__msg__RobotInstant__Sequence__fini(&msg->instants);
}

rj_msgs__msg__Trajectory *
rj_msgs__msg__Trajectory__create()
{
  rj_msgs__msg__Trajectory * msg = (rj_msgs__msg__Trajectory *)malloc(sizeof(rj_msgs__msg__Trajectory));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(rj_msgs__msg__Trajectory));
  bool success = rj_msgs__msg__Trajectory__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
rj_msgs__msg__Trajectory__destroy(rj_msgs__msg__Trajectory * msg)
{
  if (msg) {
    rj_msgs__msg__Trajectory__fini(msg);
  }
  free(msg);
}


bool
rj_msgs__msg__Trajectory__Sequence__init(rj_msgs__msg__Trajectory__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rj_msgs__msg__Trajectory * data = NULL;
  if (size) {
    data = (rj_msgs__msg__Trajectory *)calloc(size, sizeof(rj_msgs__msg__Trajectory));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = rj_msgs__msg__Trajectory__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        rj_msgs__msg__Trajectory__fini(&data[i - 1]);
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
rj_msgs__msg__Trajectory__Sequence__fini(rj_msgs__msg__Trajectory__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      rj_msgs__msg__Trajectory__fini(&array->data[i]);
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

rj_msgs__msg__Trajectory__Sequence *
rj_msgs__msg__Trajectory__Sequence__create(size_t size)
{
  rj_msgs__msg__Trajectory__Sequence * array = (rj_msgs__msg__Trajectory__Sequence *)malloc(sizeof(rj_msgs__msg__Trajectory__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = rj_msgs__msg__Trajectory__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
rj_msgs__msg__Trajectory__Sequence__destroy(rj_msgs__msg__Trajectory__Sequence * array)
{
  if (array) {
    rj_msgs__msg__Trajectory__Sequence__fini(array);
  }
  free(array);
}
