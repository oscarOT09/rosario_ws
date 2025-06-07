// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from action_msg:msg/YoloAction.idl
// generated code does not contain a copyright notice

#ifndef ACTION_MSG__MSG__DETAIL__YOLO_ACTION__STRUCT_H_
#define ACTION_MSG__MSG__DETAIL__YOLO_ACTION__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/YoloAction in the package action_msg.
typedef struct action_msg__msg__YoloAction
{
  bool rojo;
  bool amarillo;
  bool verde;
  bool adelante;
  bool girar_l;
  bool girar_r;
  bool trabajo;
  bool ceder;
  bool alto;
} action_msg__msg__YoloAction;

// Struct for a sequence of action_msg__msg__YoloAction.
typedef struct action_msg__msg__YoloAction__Sequence
{
  action_msg__msg__YoloAction * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} action_msg__msg__YoloAction__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ACTION_MSG__MSG__DETAIL__YOLO_ACTION__STRUCT_H_
