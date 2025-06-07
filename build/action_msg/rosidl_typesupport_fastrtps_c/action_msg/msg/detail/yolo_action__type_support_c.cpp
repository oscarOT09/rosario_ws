// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from action_msg:msg/YoloAction.idl
// generated code does not contain a copyright notice
#include "action_msg/msg/detail/yolo_action__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "action_msg/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "action_msg/msg/detail/yolo_action__struct.h"
#include "action_msg/msg/detail/yolo_action__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif


// forward declare type support functions


using _YoloAction__ros_msg_type = action_msg__msg__YoloAction;

static bool _YoloAction__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _YoloAction__ros_msg_type * ros_message = static_cast<const _YoloAction__ros_msg_type *>(untyped_ros_message);
  // Field name: rojo
  {
    cdr << (ros_message->rojo ? true : false);
  }

  // Field name: amarillo
  {
    cdr << (ros_message->amarillo ? true : false);
  }

  // Field name: verde
  {
    cdr << (ros_message->verde ? true : false);
  }

  // Field name: adelante
  {
    cdr << (ros_message->adelante ? true : false);
  }

  // Field name: girar_l
  {
    cdr << (ros_message->girar_l ? true : false);
  }

  // Field name: girar_r
  {
    cdr << (ros_message->girar_r ? true : false);
  }

  // Field name: trabajo
  {
    cdr << (ros_message->trabajo ? true : false);
  }

  // Field name: ceder
  {
    cdr << (ros_message->ceder ? true : false);
  }

  // Field name: alto
  {
    cdr << (ros_message->alto ? true : false);
  }

  return true;
}

static bool _YoloAction__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _YoloAction__ros_msg_type * ros_message = static_cast<_YoloAction__ros_msg_type *>(untyped_ros_message);
  // Field name: rojo
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->rojo = tmp ? true : false;
  }

  // Field name: amarillo
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->amarillo = tmp ? true : false;
  }

  // Field name: verde
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->verde = tmp ? true : false;
  }

  // Field name: adelante
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->adelante = tmp ? true : false;
  }

  // Field name: girar_l
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->girar_l = tmp ? true : false;
  }

  // Field name: girar_r
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->girar_r = tmp ? true : false;
  }

  // Field name: trabajo
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->trabajo = tmp ? true : false;
  }

  // Field name: ceder
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->ceder = tmp ? true : false;
  }

  // Field name: alto
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->alto = tmp ? true : false;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_action_msg
size_t get_serialized_size_action_msg__msg__YoloAction(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _YoloAction__ros_msg_type * ros_message = static_cast<const _YoloAction__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name rojo
  {
    size_t item_size = sizeof(ros_message->rojo);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name amarillo
  {
    size_t item_size = sizeof(ros_message->amarillo);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name verde
  {
    size_t item_size = sizeof(ros_message->verde);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name adelante
  {
    size_t item_size = sizeof(ros_message->adelante);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name girar_l
  {
    size_t item_size = sizeof(ros_message->girar_l);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name girar_r
  {
    size_t item_size = sizeof(ros_message->girar_r);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name trabajo
  {
    size_t item_size = sizeof(ros_message->trabajo);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name ceder
  {
    size_t item_size = sizeof(ros_message->ceder);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name alto
  {
    size_t item_size = sizeof(ros_message->alto);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _YoloAction__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_action_msg__msg__YoloAction(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_action_msg
size_t max_serialized_size_action_msg__msg__YoloAction(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // member: rojo
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: amarillo
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: verde
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: adelante
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: girar_l
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: girar_r
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: trabajo
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: ceder
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: alto
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = action_msg__msg__YoloAction;
    is_plain =
      (
      offsetof(DataType, alto) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _YoloAction__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_action_msg__msg__YoloAction(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_YoloAction = {
  "action_msg::msg",
  "YoloAction",
  _YoloAction__cdr_serialize,
  _YoloAction__cdr_deserialize,
  _YoloAction__get_serialized_size,
  _YoloAction__max_serialized_size
};

static rosidl_message_type_support_t _YoloAction__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_YoloAction,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, action_msg, msg, YoloAction)() {
  return &_YoloAction__type_support;
}

#if defined(__cplusplus)
}
#endif
