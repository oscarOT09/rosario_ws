// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from action_msg:msg/YoloAction.idl
// generated code does not contain a copyright notice

#ifndef ACTION_MSG__MSG__DETAIL__YOLO_ACTION__TRAITS_HPP_
#define ACTION_MSG__MSG__DETAIL__YOLO_ACTION__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "action_msg/msg/detail/yolo_action__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace action_msg
{

namespace msg
{

inline void to_flow_style_yaml(
  const YoloAction & msg,
  std::ostream & out)
{
  out << "{";
  // member: rojo
  {
    out << "rojo: ";
    rosidl_generator_traits::value_to_yaml(msg.rojo, out);
    out << ", ";
  }

  // member: amarillo
  {
    out << "amarillo: ";
    rosidl_generator_traits::value_to_yaml(msg.amarillo, out);
    out << ", ";
  }

  // member: verde
  {
    out << "verde: ";
    rosidl_generator_traits::value_to_yaml(msg.verde, out);
    out << ", ";
  }

  // member: adelante
  {
    out << "adelante: ";
    rosidl_generator_traits::value_to_yaml(msg.adelante, out);
    out << ", ";
  }

  // member: girar_l
  {
    out << "girar_l: ";
    rosidl_generator_traits::value_to_yaml(msg.girar_l, out);
    out << ", ";
  }

  // member: girar_r
  {
    out << "girar_r: ";
    rosidl_generator_traits::value_to_yaml(msg.girar_r, out);
    out << ", ";
  }

  // member: trabajo
  {
    out << "trabajo: ";
    rosidl_generator_traits::value_to_yaml(msg.trabajo, out);
    out << ", ";
  }

  // member: ceder
  {
    out << "ceder: ";
    rosidl_generator_traits::value_to_yaml(msg.ceder, out);
    out << ", ";
  }

  // member: alto
  {
    out << "alto: ";
    rosidl_generator_traits::value_to_yaml(msg.alto, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const YoloAction & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: rojo
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rojo: ";
    rosidl_generator_traits::value_to_yaml(msg.rojo, out);
    out << "\n";
  }

  // member: amarillo
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "amarillo: ";
    rosidl_generator_traits::value_to_yaml(msg.amarillo, out);
    out << "\n";
  }

  // member: verde
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "verde: ";
    rosidl_generator_traits::value_to_yaml(msg.verde, out);
    out << "\n";
  }

  // member: adelante
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "adelante: ";
    rosidl_generator_traits::value_to_yaml(msg.adelante, out);
    out << "\n";
  }

  // member: girar_l
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "girar_l: ";
    rosidl_generator_traits::value_to_yaml(msg.girar_l, out);
    out << "\n";
  }

  // member: girar_r
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "girar_r: ";
    rosidl_generator_traits::value_to_yaml(msg.girar_r, out);
    out << "\n";
  }

  // member: trabajo
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "trabajo: ";
    rosidl_generator_traits::value_to_yaml(msg.trabajo, out);
    out << "\n";
  }

  // member: ceder
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ceder: ";
    rosidl_generator_traits::value_to_yaml(msg.ceder, out);
    out << "\n";
  }

  // member: alto
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "alto: ";
    rosidl_generator_traits::value_to_yaml(msg.alto, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const YoloAction & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace action_msg

namespace rosidl_generator_traits
{

[[deprecated("use action_msg::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const action_msg::msg::YoloAction & msg,
  std::ostream & out, size_t indentation = 0)
{
  action_msg::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use action_msg::msg::to_yaml() instead")]]
inline std::string to_yaml(const action_msg::msg::YoloAction & msg)
{
  return action_msg::msg::to_yaml(msg);
}

template<>
inline const char * data_type<action_msg::msg::YoloAction>()
{
  return "action_msg::msg::YoloAction";
}

template<>
inline const char * name<action_msg::msg::YoloAction>()
{
  return "action_msg/msg/YoloAction";
}

template<>
struct has_fixed_size<action_msg::msg::YoloAction>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<action_msg::msg::YoloAction>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<action_msg::msg::YoloAction>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ACTION_MSG__MSG__DETAIL__YOLO_ACTION__TRAITS_HPP_
