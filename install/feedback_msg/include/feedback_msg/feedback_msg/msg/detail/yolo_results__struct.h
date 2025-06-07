// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from feedback_msg:msg/YoloResults.idl
// generated code does not contain a copyright notice

#ifndef FEEDBACK_MSG__MSG__DETAIL__YOLO_RESULTS__STRUCT_H_
#define FEEDBACK_MSG__MSG__DETAIL__YOLO_RESULTS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/YoloResults in the package feedback_msg.
typedef struct feedback_msg__msg__YoloResults
{
  bool dotted_line;
  int32_t signal;
  int32_t traffic_light;
} feedback_msg__msg__YoloResults;

// Struct for a sequence of feedback_msg__msg__YoloResults.
typedef struct feedback_msg__msg__YoloResults__Sequence
{
  feedback_msg__msg__YoloResults * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} feedback_msg__msg__YoloResults__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // FEEDBACK_MSG__MSG__DETAIL__YOLO_RESULTS__STRUCT_H_
