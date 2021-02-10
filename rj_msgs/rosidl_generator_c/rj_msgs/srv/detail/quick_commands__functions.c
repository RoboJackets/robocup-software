// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from rj_msgs:srv/QuickCommands.idl
// generated code does not contain a copyright notice
#include "rj_msgs/srv/detail/quick_commands__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

bool
rj_msgs__srv__QuickCommands_Request__init(rj_msgs__srv__QuickCommands_Request * msg)
{
  if (!msg) {
    return false;
  }
  // state
  return true;
}

void
rj_msgs__srv__QuickCommands_Request__fini(rj_msgs__srv__QuickCommands_Request * msg)
{
  if (!msg) {
    return;
  }
  // state
}

rj_msgs__srv__QuickCommands_Request *
rj_msgs__srv__QuickCommands_Request__create()
{
  rj_msgs__srv__QuickCommands_Request * msg = (rj_msgs__srv__QuickCommands_Request *)malloc(sizeof(rj_msgs__srv__QuickCommands_Request));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(rj_msgs__srv__QuickCommands_Request));
  bool success = rj_msgs__srv__QuickCommands_Request__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
rj_msgs__srv__QuickCommands_Request__destroy(rj_msgs__srv__QuickCommands_Request * msg)
{
  if (msg) {
    rj_msgs__srv__QuickCommands_Request__fini(msg);
  }
  free(msg);
}


bool
rj_msgs__srv__QuickCommands_Request__Sequence__init(rj_msgs__srv__QuickCommands_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rj_msgs__srv__QuickCommands_Request * data = NULL;
  if (size) {
    data = (rj_msgs__srv__QuickCommands_Request *)calloc(size, sizeof(rj_msgs__srv__QuickCommands_Request));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = rj_msgs__srv__QuickCommands_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        rj_msgs__srv__QuickCommands_Request__fini(&data[i - 1]);
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
rj_msgs__srv__QuickCommands_Request__Sequence__fini(rj_msgs__srv__QuickCommands_Request__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      rj_msgs__srv__QuickCommands_Request__fini(&array->data[i]);
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

rj_msgs__srv__QuickCommands_Request__Sequence *
rj_msgs__srv__QuickCommands_Request__Sequence__create(size_t size)
{
  rj_msgs__srv__QuickCommands_Request__Sequence * array = (rj_msgs__srv__QuickCommands_Request__Sequence *)malloc(sizeof(rj_msgs__srv__QuickCommands_Request__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = rj_msgs__srv__QuickCommands_Request__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
rj_msgs__srv__QuickCommands_Request__Sequence__destroy(rj_msgs__srv__QuickCommands_Request__Sequence * array)
{
  if (array) {
    rj_msgs__srv__QuickCommands_Request__Sequence__fini(array);
  }
  free(array);
}


bool
rj_msgs__srv__QuickCommands_Response__init(rj_msgs__srv__QuickCommands_Response * msg)
{
  if (!msg) {
    return false;
  }
  // structure_needs_at_least_one_member
  return true;
}

void
rj_msgs__srv__QuickCommands_Response__fini(rj_msgs__srv__QuickCommands_Response * msg)
{
  if (!msg) {
    return;
  }
  // structure_needs_at_least_one_member
}

rj_msgs__srv__QuickCommands_Response *
rj_msgs__srv__QuickCommands_Response__create()
{
  rj_msgs__srv__QuickCommands_Response * msg = (rj_msgs__srv__QuickCommands_Response *)malloc(sizeof(rj_msgs__srv__QuickCommands_Response));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(rj_msgs__srv__QuickCommands_Response));
  bool success = rj_msgs__srv__QuickCommands_Response__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
rj_msgs__srv__QuickCommands_Response__destroy(rj_msgs__srv__QuickCommands_Response * msg)
{
  if (msg) {
    rj_msgs__srv__QuickCommands_Response__fini(msg);
  }
  free(msg);
}


bool
rj_msgs__srv__QuickCommands_Response__Sequence__init(rj_msgs__srv__QuickCommands_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rj_msgs__srv__QuickCommands_Response * data = NULL;
  if (size) {
    data = (rj_msgs__srv__QuickCommands_Response *)calloc(size, sizeof(rj_msgs__srv__QuickCommands_Response));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = rj_msgs__srv__QuickCommands_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        rj_msgs__srv__QuickCommands_Response__fini(&data[i - 1]);
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
rj_msgs__srv__QuickCommands_Response__Sequence__fini(rj_msgs__srv__QuickCommands_Response__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      rj_msgs__srv__QuickCommands_Response__fini(&array->data[i]);
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

rj_msgs__srv__QuickCommands_Response__Sequence *
rj_msgs__srv__QuickCommands_Response__Sequence__create(size_t size)
{
  rj_msgs__srv__QuickCommands_Response__Sequence * array = (rj_msgs__srv__QuickCommands_Response__Sequence *)malloc(sizeof(rj_msgs__srv__QuickCommands_Response__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = rj_msgs__srv__QuickCommands_Response__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
rj_msgs__srv__QuickCommands_Response__Sequence__destroy(rj_msgs__srv__QuickCommands_Response__Sequence * array)
{
  if (array) {
    rj_msgs__srv__QuickCommands_Response__Sequence__fini(array);
  }
  free(array);
}
