// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from feedback_msg:msg/YoloResults.idl
// generated code does not contain a copyright notice

#ifndef FEEDBACK_MSG__MSG__DETAIL__YOLO_RESULTS__BUILDER_HPP_
#define FEEDBACK_MSG__MSG__DETAIL__YOLO_RESULTS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "feedback_msg/msg/detail/yolo_results__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace feedback_msg
{

namespace msg
{

namespace builder
{

class Init_YoloResults_traffic_light
{
public:
  explicit Init_YoloResults_traffic_light(::feedback_msg::msg::YoloResults & msg)
  : msg_(msg)
  {}
  ::feedback_msg::msg::YoloResults traffic_light(::feedback_msg::msg::YoloResults::_traffic_light_type arg)
  {
    msg_.traffic_light = std::move(arg);
    return std::move(msg_);
  }

private:
  ::feedback_msg::msg::YoloResults msg_;
};

class Init_YoloResults_signal
{
public:
  explicit Init_YoloResults_signal(::feedback_msg::msg::YoloResults & msg)
  : msg_(msg)
  {}
  Init_YoloResults_traffic_light signal(::feedback_msg::msg::YoloResults::_signal_type arg)
  {
    msg_.signal = std::move(arg);
    return Init_YoloResults_traffic_light(msg_);
  }

private:
  ::feedback_msg::msg::YoloResults msg_;
};

class Init_YoloResults_dotted_line
{
public:
  Init_YoloResults_dotted_line()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_YoloResults_signal dotted_line(::feedback_msg::msg::YoloResults::_dotted_line_type arg)
  {
    msg_.dotted_line = std::move(arg);
    return Init_YoloResults_signal(msg_);
  }

private:
  ::feedback_msg::msg::YoloResults msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::feedback_msg::msg::YoloResults>()
{
  return feedback_msg::msg::builder::Init_YoloResults_dotted_line();
}

}  // namespace feedback_msg

#endif  // FEEDBACK_MSG__MSG__DETAIL__YOLO_RESULTS__BUILDER_HPP_
