// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from rj_msgs:msg/TeamInfo.idl
// generated code does not contain a copyright notice
#include "rj_msgs/msg/detail/team_info__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


// Include directives for member types
// Member `name`
#include "rosidl_runtime_c/string_functions.h"
// Member `yellow_card_remaining_times`
// Member `remaining_timeout_time`
#include "builtin_interfaces/msg/detail/duration__functions.h"

bool
rj_msgs__msg__TeamInfo__init(rj_msgs__msg__TeamInfo * msg)
{
  if (!msg) {
    return false;
  }
  // name
  if (!rosidl_runtime_c__String__init(&msg->name)) {
    rj_msgs__msg__TeamInfo__fini(msg);
    return false;
  }
  // score
  // num_red_cards
  // num_yellow_cards
  // yellow_card_remaining_times
  if (!builtin_interfaces__msg__Duration__Sequence__init(&msg->yellow_card_remaining_times, 0)) {
    rj_msgs__msg__TeamInfo__fini(msg);
    return false;
  }
  // timeouts_left
  // remaining_timeout_time
  if (!builtin_interfaces__msg__Duration__init(&msg->remaining_timeout_time)) {
    rj_msgs__msg__TeamInfo__fini(msg);
    return false;
  }
  // goalie_id
  return true;
}

void
rj_msgs__msg__TeamInfo__fini(rj_msgs__msg__TeamInfo * msg)
{
  if (!msg) {
    return;
  }
  // name
  rosidl_runtime_c__String__fini(&msg->name);
  // score
  // num_red_cards
  // num_yellow_cards
  // yellow_card_remaining_times
  builtin_interfaces__msg__Duration__Sequence__fini(&msg->yellow_card_remaining_times);
  // timeouts_left
  // remaining_timeout_time
  builtin_interfaces__msg__Duration__fini(&msg->remaining_timeout_time);
  // goalie_id
}

rj_msgs__msg__TeamInfo *
rj_msgs__msg__TeamInfo__create()
{
  rj_msgs__msg__TeamInfo * msg = (rj_msgs__msg__TeamInfo *)malloc(sizeof(rj_msgs__msg__TeamInfo));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(rj_msgs__msg__TeamInfo));
  bool success = rj_msgs__msg__TeamInfo__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
rj_msgs__msg__TeamInfo__destroy(rj_msgs__msg__TeamInfo * msg)
{
  if (msg) {
    rj_msgs__msg__TeamInfo__fini(msg);
  }
  free(msg);
}


bool
rj_msgs__msg__TeamInfo__Sequence__init(rj_msgs__msg__TeamInfo__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rj_msgs__msg__TeamInfo * data = NULL;
  if (size) {
    data = (rj_msgs__msg__TeamInfo *)calloc(size, sizeof(rj_msgs__msg__TeamInfo));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = rj_msgs__msg__TeamInfo__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        rj_msgs__msg__TeamInfo__fini(&data[i - 1]);
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
rj_msgs__msg__TeamInfo__Sequence__fini(rj_msgs__msg__TeamInfo__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      rj_msgs__msg__TeamInfo__fini(&array->data[i]);
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

rj_msgs__msg__TeamInfo__Sequence *
rj_msgs__msg__TeamInfo__Sequence__create(size_t size)
{
  rj_msgs__msg__TeamInfo__Sequence * array = (rj_msgs__msg__TeamInfo__Sequence *)malloc(sizeof(rj_msgs__msg__TeamInfo__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = rj_msgs__msg__TeamInfo__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
rj_msgs__msg__TeamInfo__Sequence__destroy(rj_msgs__msg__TeamInfo__Sequence * array)
{
  if (array) {
    rj_msgs__msg__TeamInfo__Sequence__fini(array);
  }
  free(array);
}
