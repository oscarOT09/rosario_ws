// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rosario_path:msg/RosarioPath.idl
// generated code does not contain a copyright notice

#ifndef ROSARIO_PATH__MSG__DETAIL__ROSARIO_PATH__STRUCT_H_
#define ROSARIO_PATH__MSG__DETAIL__ROSARIO_PATH__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/RosarioPath in the package rosario_path.
typedef struct rosario_path__msg__RosarioPath
{
  float error;
  bool curva;
} rosario_path__msg__RosarioPath;

// Struct for a sequence of rosario_path__msg__RosarioPath.
typedef struct rosario_path__msg__RosarioPath__Sequence
{
  rosario_path__msg__RosarioPath * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rosario_path__msg__RosarioPath__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROSARIO_PATH__MSG__DETAIL__ROSARIO_PATH__STRUCT_H_
