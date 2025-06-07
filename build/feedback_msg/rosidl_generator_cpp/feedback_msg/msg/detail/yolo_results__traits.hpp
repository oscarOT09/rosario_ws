// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from feedback_msg:msg/YoloResults.idl
// generated code does not contain a copyright notice

#ifndef FEEDBACK_MSG__MSG__DETAIL__YOLO_RESULTS__TRAITS_HPP_
#define FEEDBACK_MSG__MSG__DETAIL__YOLO_RESULTS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "feedback_msg/msg/detail/yolo_results__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace feedback_msg
{

namespace msg
{

inline void to_flow_style_yaml(
  const YoloResults & msg,
  std::ostream & out)
{
  out << "{";
  // member: dotted_line
  {
    out << "dotted_line: ";
    rosidl_generator_traits::value_to_yaml(msg.dotted_line, out);
    out << ", ";
  }

  // member: signal
  {
    out << "signal: ";
    rosidl_generator_traits::value_to_yaml(msg.signal, out);
    out << ", ";
  }

  // member: traffic_light
  {
    out << "traffic_light: ";
    rosidl_generator_traits::value_to_yaml(msg.traffic_light, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const YoloResults & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: dotted_line
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "dotted_line: ";
    rosidl_generator_traits::value_to_yaml(msg.dotted_line, out);
    out << "\n";
  }

  // member: signal
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "signal: ";
    rosidl_generator_traits::value_to_yaml(msg.signal, out);
    out << "\n";
  }

  // member: traffic_light
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "traffic_light: ";
    rosidl_generator_traits::value_to_yaml(msg.traffic_light, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const YoloResults & msg, bool use_flow_style = false)
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

}  // namespace feedback_msg

namespace rosidl_generator_traits
{

[[deprecated("use feedback_msg::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const feedback_msg::msg::YoloResults & msg,
  std::ostream & out, size_t indentation = 0)
{
  feedback_msg::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use feedback_msg::msg::to_yaml() instead")]]
inline std::string to_yaml(const feedback_msg::msg::YoloResults & msg)
{
  return feedback_msg::msg::to_yaml(msg);
}

template<>
inline const char * data_type<feedback_msg::msg::YoloResults>()
{
  return "feedback_msg::msg::YoloResults";
}

template<>
inline const char * name<feedback_msg::msg::YoloResults>()
{
  return "feedback_msg/msg/YoloResults";
}

template<>
struct has_fixed_size<feedback_msg::msg::YoloResults>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<feedback_msg::msg::YoloResults>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<feedback_msg::msg::YoloResults>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // FEEDBACK_MSG__MSG__DETAIL__YOLO_RESULTS__TRAITS_HPP_
