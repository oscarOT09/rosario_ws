// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from feedback_msg:msg/YoloResults.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "feedback_msg/msg/detail/yolo_results__rosidl_typesupport_introspection_c.h"
#include "feedback_msg/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "feedback_msg/msg/detail/yolo_results__functions.h"
#include "feedback_msg/msg/detail/yolo_results__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void feedback_msg__msg__YoloResults__rosidl_typesupport_introspection_c__YoloResults_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  feedback_msg__msg__YoloResults__init(message_memory);
}

void feedback_msg__msg__YoloResults__rosidl_typesupport_introspection_c__YoloResults_fini_function(void * message_memory)
{
  feedback_msg__msg__YoloResults__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember feedback_msg__msg__YoloResults__rosidl_typesupport_introspection_c__YoloResults_message_member_array[3] = {
  {
    "dotted_line",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(feedback_msg__msg__YoloResults, dotted_line),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "signal",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(feedback_msg__msg__YoloResults, signal),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "traffic_light",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(feedback_msg__msg__YoloResults, traffic_light),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers feedback_msg__msg__YoloResults__rosidl_typesupport_introspection_c__YoloResults_message_members = {
  "feedback_msg__msg",  // message namespace
  "YoloResults",  // message name
  3,  // number of fields
  sizeof(feedback_msg__msg__YoloResults),
  feedback_msg__msg__YoloResults__rosidl_typesupport_introspection_c__YoloResults_message_member_array,  // message members
  feedback_msg__msg__YoloResults__rosidl_typesupport_introspection_c__YoloResults_init_function,  // function to initialize message memory (memory has to be allocated)
  feedback_msg__msg__YoloResults__rosidl_typesupport_introspection_c__YoloResults_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t feedback_msg__msg__YoloResults__rosidl_typesupport_introspection_c__YoloResults_message_type_support_handle = {
  0,
  &feedback_msg__msg__YoloResults__rosidl_typesupport_introspection_c__YoloResults_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_feedback_msg
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, feedback_msg, msg, YoloResults)() {
  if (!feedback_msg__msg__YoloResults__rosidl_typesupport_introspection_c__YoloResults_message_type_support_handle.typesupport_identifier) {
    feedback_msg__msg__YoloResults__rosidl_typesupport_introspection_c__YoloResults_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &feedback_msg__msg__YoloResults__rosidl_typesupport_introspection_c__YoloResults_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
