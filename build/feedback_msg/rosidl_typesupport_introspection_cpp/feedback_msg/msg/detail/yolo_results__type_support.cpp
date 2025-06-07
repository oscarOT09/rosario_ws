// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from feedback_msg:msg/YoloResults.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "feedback_msg/msg/detail/yolo_results__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace feedback_msg
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void YoloResults_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) feedback_msg::msg::YoloResults(_init);
}

void YoloResults_fini_function(void * message_memory)
{
  auto typed_message = static_cast<feedback_msg::msg::YoloResults *>(message_memory);
  typed_message->~YoloResults();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember YoloResults_message_member_array[3] = {
  {
    "dotted_line",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(feedback_msg::msg::YoloResults, dotted_line),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "signal",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(feedback_msg::msg::YoloResults, signal),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "traffic_light",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(feedback_msg::msg::YoloResults, traffic_light),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers YoloResults_message_members = {
  "feedback_msg::msg",  // message namespace
  "YoloResults",  // message name
  3,  // number of fields
  sizeof(feedback_msg::msg::YoloResults),
  YoloResults_message_member_array,  // message members
  YoloResults_init_function,  // function to initialize message memory (memory has to be allocated)
  YoloResults_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t YoloResults_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &YoloResults_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace feedback_msg


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<feedback_msg::msg::YoloResults>()
{
  return &::feedback_msg::msg::rosidl_typesupport_introspection_cpp::YoloResults_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, feedback_msg, msg, YoloResults)() {
  return &::feedback_msg::msg::rosidl_typesupport_introspection_cpp::YoloResults_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
