// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from action_msg:msg/YoloAction.idl
// generated code does not contain a copyright notice
#include "action_msg/msg/detail/yolo_action__rosidl_typesupport_fastrtps_cpp.hpp"
#include "action_msg/msg/detail/yolo_action__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions

namespace action_msg
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_action_msg
cdr_serialize(
  const action_msg::msg::YoloAction & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: rojo
  cdr << (ros_message.rojo ? true : false);
  // Member: amarillo
  cdr << (ros_message.amarillo ? true : false);
  // Member: verde
  cdr << (ros_message.verde ? true : false);
  // Member: adelante
  cdr << (ros_message.adelante ? true : false);
  // Member: girar_l
  cdr << (ros_message.girar_l ? true : false);
  // Member: girar_r
  cdr << (ros_message.girar_r ? true : false);
  // Member: trabajo
  cdr << (ros_message.trabajo ? true : false);
  // Member: ceder
  cdr << (ros_message.ceder ? true : false);
  // Member: alto
  cdr << (ros_message.alto ? true : false);
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_action_msg
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  action_msg::msg::YoloAction & ros_message)
{
  // Member: rojo
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.rojo = tmp ? true : false;
  }

  // Member: amarillo
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.amarillo = tmp ? true : false;
  }

  // Member: verde
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.verde = tmp ? true : false;
  }

  // Member: adelante
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.adelante = tmp ? true : false;
  }

  // Member: girar_l
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.girar_l = tmp ? true : false;
  }

  // Member: girar_r
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.girar_r = tmp ? true : false;
  }

  // Member: trabajo
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.trabajo = tmp ? true : false;
  }

  // Member: ceder
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.ceder = tmp ? true : false;
  }

  // Member: alto
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.alto = tmp ? true : false;
  }

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_action_msg
get_serialized_size(
  const action_msg::msg::YoloAction & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: rojo
  {
    size_t item_size = sizeof(ros_message.rojo);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: amarillo
  {
    size_t item_size = sizeof(ros_message.amarillo);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: verde
  {
    size_t item_size = sizeof(ros_message.verde);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: adelante
  {
    size_t item_size = sizeof(ros_message.adelante);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: girar_l
  {
    size_t item_size = sizeof(ros_message.girar_l);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: girar_r
  {
    size_t item_size = sizeof(ros_message.girar_r);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: trabajo
  {
    size_t item_size = sizeof(ros_message.trabajo);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: ceder
  {
    size_t item_size = sizeof(ros_message.ceder);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: alto
  {
    size_t item_size = sizeof(ros_message.alto);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_action_msg
max_serialized_size_YoloAction(
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


  // Member: rojo
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: amarillo
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: verde
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: adelante
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: girar_l
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: girar_r
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: trabajo
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: ceder
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: alto
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
    using DataType = action_msg::msg::YoloAction;
    is_plain =
      (
      offsetof(DataType, alto) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _YoloAction__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const action_msg::msg::YoloAction *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _YoloAction__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<action_msg::msg::YoloAction *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _YoloAction__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const action_msg::msg::YoloAction *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _YoloAction__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_YoloAction(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _YoloAction__callbacks = {
  "action_msg::msg",
  "YoloAction",
  _YoloAction__cdr_serialize,
  _YoloAction__cdr_deserialize,
  _YoloAction__get_serialized_size,
  _YoloAction__max_serialized_size
};

static rosidl_message_type_support_t _YoloAction__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_YoloAction__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace action_msg

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_action_msg
const rosidl_message_type_support_t *
get_message_type_support_handle<action_msg::msg::YoloAction>()
{
  return &action_msg::msg::typesupport_fastrtps_cpp::_YoloAction__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, action_msg, msg, YoloAction)() {
  return &action_msg::msg::typesupport_fastrtps_cpp::_YoloAction__handle;
}

#ifdef __cplusplus
}
#endif
