// generated from rosidl_typesupport_c/resource/idl__type_support.cpp.em
// with input from action_msg:msg/YoloAction.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "action_msg/msg/detail/yolo_action__struct.h"
#include "action_msg/msg/detail/yolo_action__type_support.h"
#include "rosidl_typesupport_c/identifier.h"
#include "rosidl_typesupport_c/message_type_support_dispatch.h"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_c/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace action_msg
{

namespace msg
{

namespace rosidl_typesupport_c
{

typedef struct _YoloAction_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _YoloAction_type_support_ids_t;

static const _YoloAction_type_support_ids_t _YoloAction_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _YoloAction_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _YoloAction_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _YoloAction_type_support_symbol_names_t _YoloAction_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, action_msg, msg, YoloAction)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, action_msg, msg, YoloAction)),
  }
};

typedef struct _YoloAction_type_support_data_t
{
  void * data[2];
} _YoloAction_type_support_data_t;

static _YoloAction_type_support_data_t _YoloAction_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _YoloAction_message_typesupport_map = {
  2,
  "action_msg",
  &_YoloAction_message_typesupport_ids.typesupport_identifier[0],
  &_YoloAction_message_typesupport_symbol_names.symbol_name[0],
  &_YoloAction_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t YoloAction_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_YoloAction_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace msg

}  // namespace action_msg

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, action_msg, msg, YoloAction)() {
  return &::action_msg::msg::rosidl_typesupport_c::YoloAction_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
