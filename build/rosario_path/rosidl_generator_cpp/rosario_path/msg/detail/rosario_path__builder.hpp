// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rosario_path:msg/RosarioPath.idl
// generated code does not contain a copyright notice

#ifndef ROSARIO_PATH__MSG__DETAIL__ROSARIO_PATH__BUILDER_HPP_
#define ROSARIO_PATH__MSG__DETAIL__ROSARIO_PATH__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "rosario_path/msg/detail/rosario_path__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace rosario_path
{

namespace msg
{

namespace builder
{

class Init_RosarioPath_curva
{
public:
  explicit Init_RosarioPath_curva(::rosario_path::msg::RosarioPath & msg)
  : msg_(msg)
  {}
  ::rosario_path::msg::RosarioPath curva(::rosario_path::msg::RosarioPath::_curva_type arg)
  {
    msg_.curva = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rosario_path::msg::RosarioPath msg_;
};

class Init_RosarioPath_error
{
public:
  Init_RosarioPath_error()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RosarioPath_curva error(::rosario_path::msg::RosarioPath::_error_type arg)
  {
    msg_.error = std::move(arg);
    return Init_RosarioPath_curva(msg_);
  }

private:
  ::rosario_path::msg::RosarioPath msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::rosario_path::msg::RosarioPath>()
{
  return rosario_path::msg::builder::Init_RosarioPath_error();
}

}  // namespace rosario_path

#endif  // ROSARIO_PATH__MSG__DETAIL__ROSARIO_PATH__BUILDER_HPP_
