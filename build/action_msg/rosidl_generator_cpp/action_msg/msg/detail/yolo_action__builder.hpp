// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from action_msg:msg/YoloAction.idl
// generated code does not contain a copyright notice

#ifndef ACTION_MSG__MSG__DETAIL__YOLO_ACTION__BUILDER_HPP_
#define ACTION_MSG__MSG__DETAIL__YOLO_ACTION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "action_msg/msg/detail/yolo_action__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace action_msg
{

namespace msg
{

namespace builder
{

class Init_YoloAction_alto
{
public:
  explicit Init_YoloAction_alto(::action_msg::msg::YoloAction & msg)
  : msg_(msg)
  {}
  ::action_msg::msg::YoloAction alto(::action_msg::msg::YoloAction::_alto_type arg)
  {
    msg_.alto = std::move(arg);
    return std::move(msg_);
  }

private:
  ::action_msg::msg::YoloAction msg_;
};

class Init_YoloAction_ceder
{
public:
  explicit Init_YoloAction_ceder(::action_msg::msg::YoloAction & msg)
  : msg_(msg)
  {}
  Init_YoloAction_alto ceder(::action_msg::msg::YoloAction::_ceder_type arg)
  {
    msg_.ceder = std::move(arg);
    return Init_YoloAction_alto(msg_);
  }

private:
  ::action_msg::msg::YoloAction msg_;
};

class Init_YoloAction_trabajo
{
public:
  explicit Init_YoloAction_trabajo(::action_msg::msg::YoloAction & msg)
  : msg_(msg)
  {}
  Init_YoloAction_ceder trabajo(::action_msg::msg::YoloAction::_trabajo_type arg)
  {
    msg_.trabajo = std::move(arg);
    return Init_YoloAction_ceder(msg_);
  }

private:
  ::action_msg::msg::YoloAction msg_;
};

class Init_YoloAction_girar_r
{
public:
  explicit Init_YoloAction_girar_r(::action_msg::msg::YoloAction & msg)
  : msg_(msg)
  {}
  Init_YoloAction_trabajo girar_r(::action_msg::msg::YoloAction::_girar_r_type arg)
  {
    msg_.girar_r = std::move(arg);
    return Init_YoloAction_trabajo(msg_);
  }

private:
  ::action_msg::msg::YoloAction msg_;
};

class Init_YoloAction_girar_l
{
public:
  explicit Init_YoloAction_girar_l(::action_msg::msg::YoloAction & msg)
  : msg_(msg)
  {}
  Init_YoloAction_girar_r girar_l(::action_msg::msg::YoloAction::_girar_l_type arg)
  {
    msg_.girar_l = std::move(arg);
    return Init_YoloAction_girar_r(msg_);
  }

private:
  ::action_msg::msg::YoloAction msg_;
};

class Init_YoloAction_adelante
{
public:
  explicit Init_YoloAction_adelante(::action_msg::msg::YoloAction & msg)
  : msg_(msg)
  {}
  Init_YoloAction_girar_l adelante(::action_msg::msg::YoloAction::_adelante_type arg)
  {
    msg_.adelante = std::move(arg);
    return Init_YoloAction_girar_l(msg_);
  }

private:
  ::action_msg::msg::YoloAction msg_;
};

class Init_YoloAction_verde
{
public:
  explicit Init_YoloAction_verde(::action_msg::msg::YoloAction & msg)
  : msg_(msg)
  {}
  Init_YoloAction_adelante verde(::action_msg::msg::YoloAction::_verde_type arg)
  {
    msg_.verde = std::move(arg);
    return Init_YoloAction_adelante(msg_);
  }

private:
  ::action_msg::msg::YoloAction msg_;
};

class Init_YoloAction_amarillo
{
public:
  explicit Init_YoloAction_amarillo(::action_msg::msg::YoloAction & msg)
  : msg_(msg)
  {}
  Init_YoloAction_verde amarillo(::action_msg::msg::YoloAction::_amarillo_type arg)
  {
    msg_.amarillo = std::move(arg);
    return Init_YoloAction_verde(msg_);
  }

private:
  ::action_msg::msg::YoloAction msg_;
};

class Init_YoloAction_rojo
{
public:
  Init_YoloAction_rojo()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_YoloAction_amarillo rojo(::action_msg::msg::YoloAction::_rojo_type arg)
  {
    msg_.rojo = std::move(arg);
    return Init_YoloAction_amarillo(msg_);
  }

private:
  ::action_msg::msg::YoloAction msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::action_msg::msg::YoloAction>()
{
  return action_msg::msg::builder::Init_YoloAction_rojo();
}

}  // namespace action_msg

#endif  // ACTION_MSG__MSG__DETAIL__YOLO_ACTION__BUILDER_HPP_
