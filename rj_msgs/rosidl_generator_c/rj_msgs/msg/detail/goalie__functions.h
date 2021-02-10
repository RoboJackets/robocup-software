// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from rj_msgs:msg/Goalie.idl
// generated code does not contain a copyright notice

#ifndef RJ_MSGS__MSG__DETAIL__GOALIE__FUNCTIONS_H_
#define RJ_MSGS__MSG__DETAIL__GOALIE__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "rj_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "rj_msgs/msg/detail/goalie__struct.h"

/// Initialize msg/Goalie message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * rj_msgs__msg__Goalie
 * )) before or use
 * rj_msgs__msg__Goalie__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_rj_msgs
bool
rj_msgs__msg__Goalie__init(rj_msgs__msg__Goalie * msg);

/// Finalize msg/Goalie message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rj_msgs
void
rj_msgs__msg__Goalie__fini(rj_msgs__msg__Goalie * msg);

/// Create msg/Goalie message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * rj_msgs__msg__Goalie__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_rj_msgs
rj_msgs__msg__Goalie *
rj_msgs__msg__Goalie__create();

/// Destroy msg/Goalie message.
/**
 * It calls
 * rj_msgs__msg__Goalie__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rj_msgs
void
rj_msgs__msg__Goalie__destroy(rj_msgs__msg__Goalie * msg);


/// Initialize array of msg/Goalie messages.
/**
 * It allocates the memory for the number of elements and calls
 * rj_msgs__msg__Goalie__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_rj_msgs
bool
rj_msgs__msg__Goalie__Sequence__init(rj_msgs__msg__Goalie__Sequence * array, size_t size);

/// Finalize array of msg/Goalie messages.
/**
 * It calls
 * rj_msgs__msg__Goalie__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rj_msgs
void
rj_msgs__msg__Goalie__Sequence__fini(rj_msgs__msg__Goalie__Sequence * array);

/// Create array of msg/Goalie messages.
/**
 * It allocates the memory for the array and calls
 * rj_msgs__msg__Goalie__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_rj_msgs
rj_msgs__msg__Goalie__Sequence *
rj_msgs__msg__Goalie__Sequence__create(size_t size);

/// Destroy array of msg/Goalie messages.
/**
 * It calls
 * rj_msgs__msg__Goalie__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rj_msgs
void
rj_msgs__msg__Goalie__Sequence__destroy(rj_msgs__msg__Goalie__Sequence * array);

#ifdef __cplusplus
}
#endif

#endif  // RJ_MSGS__MSG__DETAIL__GOALIE__FUNCTIONS_H_
