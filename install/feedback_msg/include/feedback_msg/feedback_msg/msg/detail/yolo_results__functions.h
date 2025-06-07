// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from feedback_msg:msg/YoloResults.idl
// generated code does not contain a copyright notice

#ifndef FEEDBACK_MSG__MSG__DETAIL__YOLO_RESULTS__FUNCTIONS_H_
#define FEEDBACK_MSG__MSG__DETAIL__YOLO_RESULTS__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "feedback_msg/msg/rosidl_generator_c__visibility_control.h"

#include "feedback_msg/msg/detail/yolo_results__struct.h"

/// Initialize msg/YoloResults message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * feedback_msg__msg__YoloResults
 * )) before or use
 * feedback_msg__msg__YoloResults__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_feedback_msg
bool
feedback_msg__msg__YoloResults__init(feedback_msg__msg__YoloResults * msg);

/// Finalize msg/YoloResults message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_feedback_msg
void
feedback_msg__msg__YoloResults__fini(feedback_msg__msg__YoloResults * msg);

/// Create msg/YoloResults message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * feedback_msg__msg__YoloResults__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_feedback_msg
feedback_msg__msg__YoloResults *
feedback_msg__msg__YoloResults__create();

/// Destroy msg/YoloResults message.
/**
 * It calls
 * feedback_msg__msg__YoloResults__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_feedback_msg
void
feedback_msg__msg__YoloResults__destroy(feedback_msg__msg__YoloResults * msg);

/// Check for msg/YoloResults message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_feedback_msg
bool
feedback_msg__msg__YoloResults__are_equal(const feedback_msg__msg__YoloResults * lhs, const feedback_msg__msg__YoloResults * rhs);

/// Copy a msg/YoloResults message.
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
ROSIDL_GENERATOR_C_PUBLIC_feedback_msg
bool
feedback_msg__msg__YoloResults__copy(
  const feedback_msg__msg__YoloResults * input,
  feedback_msg__msg__YoloResults * output);

/// Initialize array of msg/YoloResults messages.
/**
 * It allocates the memory for the number of elements and calls
 * feedback_msg__msg__YoloResults__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_feedback_msg
bool
feedback_msg__msg__YoloResults__Sequence__init(feedback_msg__msg__YoloResults__Sequence * array, size_t size);

/// Finalize array of msg/YoloResults messages.
/**
 * It calls
 * feedback_msg__msg__YoloResults__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_feedback_msg
void
feedback_msg__msg__YoloResults__Sequence__fini(feedback_msg__msg__YoloResults__Sequence * array);

/// Create array of msg/YoloResults messages.
/**
 * It allocates the memory for the array and calls
 * feedback_msg__msg__YoloResults__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_feedback_msg
feedback_msg__msg__YoloResults__Sequence *
feedback_msg__msg__YoloResults__Sequence__create(size_t size);

/// Destroy array of msg/YoloResults messages.
/**
 * It calls
 * feedback_msg__msg__YoloResults__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_feedback_msg
void
feedback_msg__msg__YoloResults__Sequence__destroy(feedback_msg__msg__YoloResults__Sequence * array);

/// Check for msg/YoloResults message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_feedback_msg
bool
feedback_msg__msg__YoloResults__Sequence__are_equal(const feedback_msg__msg__YoloResults__Sequence * lhs, const feedback_msg__msg__YoloResults__Sequence * rhs);

/// Copy an array of msg/YoloResults messages.
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
ROSIDL_GENERATOR_C_PUBLIC_feedback_msg
bool
feedback_msg__msg__YoloResults__Sequence__copy(
  const feedback_msg__msg__YoloResults__Sequence * input,
  feedback_msg__msg__YoloResults__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // FEEDBACK_MSG__MSG__DETAIL__YOLO_RESULTS__FUNCTIONS_H_
