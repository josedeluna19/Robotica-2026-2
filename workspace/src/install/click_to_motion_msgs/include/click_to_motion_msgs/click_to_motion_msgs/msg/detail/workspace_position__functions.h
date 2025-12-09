// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from click_to_motion_msgs:msg/WorkspacePosition.idl
// generated code does not contain a copyright notice

#ifndef CLICK_TO_MOTION_MSGS__MSG__DETAIL__WORKSPACE_POSITION__FUNCTIONS_H_
#define CLICK_TO_MOTION_MSGS__MSG__DETAIL__WORKSPACE_POSITION__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "click_to_motion_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "click_to_motion_msgs/msg/detail/workspace_position__struct.h"

/// Initialize msg/WorkspacePosition message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * click_to_motion_msgs__msg__WorkspacePosition
 * )) before or use
 * click_to_motion_msgs__msg__WorkspacePosition__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_click_to_motion_msgs
bool
click_to_motion_msgs__msg__WorkspacePosition__init(click_to_motion_msgs__msg__WorkspacePosition * msg);

/// Finalize msg/WorkspacePosition message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_click_to_motion_msgs
void
click_to_motion_msgs__msg__WorkspacePosition__fini(click_to_motion_msgs__msg__WorkspacePosition * msg);

/// Create msg/WorkspacePosition message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * click_to_motion_msgs__msg__WorkspacePosition__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_click_to_motion_msgs
click_to_motion_msgs__msg__WorkspacePosition *
click_to_motion_msgs__msg__WorkspacePosition__create();

/// Destroy msg/WorkspacePosition message.
/**
 * It calls
 * click_to_motion_msgs__msg__WorkspacePosition__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_click_to_motion_msgs
void
click_to_motion_msgs__msg__WorkspacePosition__destroy(click_to_motion_msgs__msg__WorkspacePosition * msg);

/// Check for msg/WorkspacePosition message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_click_to_motion_msgs
bool
click_to_motion_msgs__msg__WorkspacePosition__are_equal(const click_to_motion_msgs__msg__WorkspacePosition * lhs, const click_to_motion_msgs__msg__WorkspacePosition * rhs);

/// Copy a msg/WorkspacePosition message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_click_to_motion_msgs
bool
click_to_motion_msgs__msg__WorkspacePosition__copy(
  const click_to_motion_msgs__msg__WorkspacePosition * input,
  click_to_motion_msgs__msg__WorkspacePosition * output);

/// Initialize array of msg/WorkspacePosition messages.
/**
 * It allocates the memory for the number of elements and calls
 * click_to_motion_msgs__msg__WorkspacePosition__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_click_to_motion_msgs
bool
click_to_motion_msgs__msg__WorkspacePosition__Sequence__init(click_to_motion_msgs__msg__WorkspacePosition__Sequence * array, size_t size);

/// Finalize array of msg/WorkspacePosition messages.
/**
 * It calls
 * click_to_motion_msgs__msg__WorkspacePosition__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_click_to_motion_msgs
void
click_to_motion_msgs__msg__WorkspacePosition__Sequence__fini(click_to_motion_msgs__msg__WorkspacePosition__Sequence * array);

/// Create array of msg/WorkspacePosition messages.
/**
 * It allocates the memory for the array and calls
 * click_to_motion_msgs__msg__WorkspacePosition__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_click_to_motion_msgs
click_to_motion_msgs__msg__WorkspacePosition__Sequence *
click_to_motion_msgs__msg__WorkspacePosition__Sequence__create(size_t size);

/// Destroy array of msg/WorkspacePosition messages.
/**
 * It calls
 * click_to_motion_msgs__msg__WorkspacePosition__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_click_to_motion_msgs
void
click_to_motion_msgs__msg__WorkspacePosition__Sequence__destroy(click_to_motion_msgs__msg__WorkspacePosition__Sequence * array);

/// Check for msg/WorkspacePosition message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_click_to_motion_msgs
bool
click_to_motion_msgs__msg__WorkspacePosition__Sequence__are_equal(const click_to_motion_msgs__msg__WorkspacePosition__Sequence * lhs, const click_to_motion_msgs__msg__WorkspacePosition__Sequence * rhs);

/// Copy an array of msg/WorkspacePosition messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_click_to_motion_msgs
bool
click_to_motion_msgs__msg__WorkspacePosition__Sequence__copy(
  const click_to_motion_msgs__msg__WorkspacePosition__Sequence * input,
  click_to_motion_msgs__msg__WorkspacePosition__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // CLICK_TO_MOTION_MSGS__MSG__DETAIL__WORKSPACE_POSITION__FUNCTIONS_H_
