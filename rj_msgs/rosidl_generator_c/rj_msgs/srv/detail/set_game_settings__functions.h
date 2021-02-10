// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from rj_msgs:srv/SetGameSettings.idl
// generated code does not contain a copyright notice

#ifndef RJ_MSGS__SRV__DETAIL__SET_GAME_SETTINGS__FUNCTIONS_H_
#define RJ_MSGS__SRV__DETAIL__SET_GAME_SETTINGS__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "rj_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "rj_msgs/srv/detail/set_game_settings__struct.h"

/// Initialize srv/SetGameSettings message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * rj_msgs__srv__SetGameSettings_Request
 * )) before or use
 * rj_msgs__srv__SetGameSettings_Request__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_rj_msgs
bool
rj_msgs__srv__SetGameSettings_Request__init(rj_msgs__srv__SetGameSettings_Request * msg);

/// Finalize srv/SetGameSettings message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rj_msgs
void
rj_msgs__srv__SetGameSettings_Request__fini(rj_msgs__srv__SetGameSettings_Request * msg);

/// Create srv/SetGameSettings message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * rj_msgs__srv__SetGameSettings_Request__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_rj_msgs
rj_msgs__srv__SetGameSettings_Request *
rj_msgs__srv__SetGameSettings_Request__create();

/// Destroy srv/SetGameSettings message.
/**
 * It calls
 * rj_msgs__srv__SetGameSettings_Request__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rj_msgs
void
rj_msgs__srv__SetGameSettings_Request__destroy(rj_msgs__srv__SetGameSettings_Request * msg);


/// Initialize array of srv/SetGameSettings messages.
/**
 * It allocates the memory for the number of elements and calls
 * rj_msgs__srv__SetGameSettings_Request__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_rj_msgs
bool
rj_msgs__srv__SetGameSettings_Request__Sequence__init(rj_msgs__srv__SetGameSettings_Request__Sequence * array, size_t size);

/// Finalize array of srv/SetGameSettings messages.
/**
 * It calls
 * rj_msgs__srv__SetGameSettings_Request__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rj_msgs
void
rj_msgs__srv__SetGameSettings_Request__Sequence__fini(rj_msgs__srv__SetGameSettings_Request__Sequence * array);

/// Create array of srv/SetGameSettings messages.
/**
 * It allocates the memory for the array and calls
 * rj_msgs__srv__SetGameSettings_Request__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_rj_msgs
rj_msgs__srv__SetGameSettings_Request__Sequence *
rj_msgs__srv__SetGameSettings_Request__Sequence__create(size_t size);

/// Destroy array of srv/SetGameSettings messages.
/**
 * It calls
 * rj_msgs__srv__SetGameSettings_Request__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rj_msgs
void
rj_msgs__srv__SetGameSettings_Request__Sequence__destroy(rj_msgs__srv__SetGameSettings_Request__Sequence * array);

/// Initialize srv/SetGameSettings message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * rj_msgs__srv__SetGameSettings_Response
 * )) before or use
 * rj_msgs__srv__SetGameSettings_Response__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_rj_msgs
bool
rj_msgs__srv__SetGameSettings_Response__init(rj_msgs__srv__SetGameSettings_Response * msg);

/// Finalize srv/SetGameSettings message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rj_msgs
void
rj_msgs__srv__SetGameSettings_Response__fini(rj_msgs__srv__SetGameSettings_Response * msg);

/// Create srv/SetGameSettings message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * rj_msgs__srv__SetGameSettings_Response__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_rj_msgs
rj_msgs__srv__SetGameSettings_Response *
rj_msgs__srv__SetGameSettings_Response__create();

/// Destroy srv/SetGameSettings message.
/**
 * It calls
 * rj_msgs__srv__SetGameSettings_Response__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rj_msgs
void
rj_msgs__srv__SetGameSettings_Response__destroy(rj_msgs__srv__SetGameSettings_Response * msg);


/// Initialize array of srv/SetGameSettings messages.
/**
 * It allocates the memory for the number of elements and calls
 * rj_msgs__srv__SetGameSettings_Response__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_rj_msgs
bool
rj_msgs__srv__SetGameSettings_Response__Sequence__init(rj_msgs__srv__SetGameSettings_Response__Sequence * array, size_t size);

/// Finalize array of srv/SetGameSettings messages.
/**
 * It calls
 * rj_msgs__srv__SetGameSettings_Response__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rj_msgs
void
rj_msgs__srv__SetGameSettings_Response__Sequence__fini(rj_msgs__srv__SetGameSettings_Response__Sequence * array);

/// Create array of srv/SetGameSettings messages.
/**
 * It allocates the memory for the array and calls
 * rj_msgs__srv__SetGameSettings_Response__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_rj_msgs
rj_msgs__srv__SetGameSettings_Response__Sequence *
rj_msgs__srv__SetGameSettings_Response__Sequence__create(size_t size);

/// Destroy array of srv/SetGameSettings messages.
/**
 * It calls
 * rj_msgs__srv__SetGameSettings_Response__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rj_msgs
void
rj_msgs__srv__SetGameSettings_Response__Sequence__destroy(rj_msgs__srv__SetGameSettings_Response__Sequence * array);

#ifdef __cplusplus
}
#endif

#endif  // RJ_MSGS__SRV__DETAIL__SET_GAME_SETTINGS__FUNCTIONS_H_
