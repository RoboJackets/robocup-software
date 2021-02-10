// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from rj_msgs:msg/WorldState.idl
// generated code does not contain a copyright notice

#ifndef RJ_MSGS__MSG__DETAIL__WORLD_STATE__FUNCTIONS_H_
#define RJ_MSGS__MSG__DETAIL__WORLD_STATE__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "rj_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "rj_msgs/msg/detail/world_state__struct.h"

/// Initialize msg/WorldState message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * rj_msgs__msg__WorldState
 * )) before or use
 * rj_msgs__msg__WorldState__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_rj_msgs
bool
rj_msgs__msg__WorldState__init(rj_msgs__msg__WorldState * msg);

/// Finalize msg/WorldState message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rj_msgs
void
rj_msgs__msg__WorldState__fini(rj_msgs__msg__WorldState * msg);

/// Create msg/WorldState message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * rj_msgs__msg__WorldState__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_rj_msgs
rj_msgs__msg__WorldState *
rj_msgs__msg__WorldState__create();

/// Destroy msg/WorldState message.
/**
 * It calls
 * rj_msgs__msg__WorldState__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rj_msgs
void
rj_msgs__msg__WorldState__destroy(rj_msgs__msg__WorldState * msg);


/// Initialize array of msg/WorldState messages.
/**
 * It allocates the memory for the number of elements and calls
 * rj_msgs__msg__WorldState__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_rj_msgs
bool
rj_msgs__msg__WorldState__Sequence__init(rj_msgs__msg__WorldState__Sequence * array, size_t size);

/// Finalize array of msg/WorldState messages.
/**
 * It calls
 * rj_msgs__msg__WorldState__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rj_msgs
void
rj_msgs__msg__WorldState__Sequence__fini(rj_msgs__msg__WorldState__Sequence * array);

/// Create array of msg/WorldState messages.
/**
 * It allocates the memory for the array and calls
 * rj_msgs__msg__WorldState__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_rj_msgs
rj_msgs__msg__WorldState__Sequence *
rj_msgs__msg__WorldState__Sequence__create(size_t size);

/// Destroy array of msg/WorldState messages.
/**
 * It calls
 * rj_msgs__msg__WorldState__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rj_msgs
void
rj_msgs__msg__WorldState__Sequence__destroy(rj_msgs__msg__WorldState__Sequence * array);

#ifdef __cplusplus
}
#endif

#endif  // RJ_MSGS__MSG__DETAIL__WORLD_STATE__FUNCTIONS_H_
