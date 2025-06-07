// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from action_msg:msg/YoloAction.idl
// generated code does not contain a copyright notice

#ifndef ACTION_MSG__MSG__DETAIL__YOLO_ACTION__STRUCT_HPP_
#define ACTION_MSG__MSG__DETAIL__YOLO_ACTION__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__action_msg__msg__YoloAction __attribute__((deprecated))
#else
# define DEPRECATED__action_msg__msg__YoloAction __declspec(deprecated)
#endif

namespace action_msg
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct YoloAction_
{
  using Type = YoloAction_<ContainerAllocator>;

  explicit YoloAction_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->rojo = false;
      this->amarillo = false;
      this->verde = false;
      this->adelante = false;
      this->girar_l = false;
      this->girar_r = false;
      this->trabajo = false;
      this->ceder = false;
      this->alto = false;
    }
  }

  explicit YoloAction_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->rojo = false;
      this->amarillo = false;
      this->verde = false;
      this->adelante = false;
      this->girar_l = false;
      this->girar_r = false;
      this->trabajo = false;
      this->ceder = false;
      this->alto = false;
    }
  }

  // field types and members
  using _rojo_type =
    bool;
  _rojo_type rojo;
  using _amarillo_type =
    bool;
  _amarillo_type amarillo;
  using _verde_type =
    bool;
  _verde_type verde;
  using _adelante_type =
    bool;
  _adelante_type adelante;
  using _girar_l_type =
    bool;
  _girar_l_type girar_l;
  using _girar_r_type =
    bool;
  _girar_r_type girar_r;
  using _trabajo_type =
    bool;
  _trabajo_type trabajo;
  using _ceder_type =
    bool;
  _ceder_type ceder;
  using _alto_type =
    bool;
  _alto_type alto;

  // setters for named parameter idiom
  Type & set__rojo(
    const bool & _arg)
  {
    this->rojo = _arg;
    return *this;
  }
  Type & set__amarillo(
    const bool & _arg)
  {
    this->amarillo = _arg;
    return *this;
  }
  Type & set__verde(
    const bool & _arg)
  {
    this->verde = _arg;
    return *this;
  }
  Type & set__adelante(
    const bool & _arg)
  {
    this->adelante = _arg;
    return *this;
  }
  Type & set__girar_l(
    const bool & _arg)
  {
    this->girar_l = _arg;
    return *this;
  }
  Type & set__girar_r(
    const bool & _arg)
  {
    this->girar_r = _arg;
    return *this;
  }
  Type & set__trabajo(
    const bool & _arg)
  {
    this->trabajo = _arg;
    return *this;
  }
  Type & set__ceder(
    const bool & _arg)
  {
    this->ceder = _arg;
    return *this;
  }
  Type & set__alto(
    const bool & _arg)
  {
    this->alto = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    action_msg::msg::YoloAction_<ContainerAllocator> *;
  using ConstRawPtr =
    const action_msg::msg::YoloAction_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<action_msg::msg::YoloAction_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<action_msg::msg::YoloAction_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      action_msg::msg::YoloAction_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<action_msg::msg::YoloAction_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      action_msg::msg::YoloAction_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<action_msg::msg::YoloAction_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<action_msg::msg::YoloAction_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<action_msg::msg::YoloAction_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__action_msg__msg__YoloAction
    std::shared_ptr<action_msg::msg::YoloAction_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__action_msg__msg__YoloAction
    std::shared_ptr<action_msg::msg::YoloAction_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const YoloAction_ & other) const
  {
    if (this->rojo != other.rojo) {
      return false;
    }
    if (this->amarillo != other.amarillo) {
      return false;
    }
    if (this->verde != other.verde) {
      return false;
    }
    if (this->adelante != other.adelante) {
      return false;
    }
    if (this->girar_l != other.girar_l) {
      return false;
    }
    if (this->girar_r != other.girar_r) {
      return false;
    }
    if (this->trabajo != other.trabajo) {
      return false;
    }
    if (this->ceder != other.ceder) {
      return false;
    }
    if (this->alto != other.alto) {
      return false;
    }
    return true;
  }
  bool operator!=(const YoloAction_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct YoloAction_

// alias to use template instance with default allocator
using YoloAction =
  action_msg::msg::YoloAction_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace action_msg

#endif  // ACTION_MSG__MSG__DETAIL__YOLO_ACTION__STRUCT_HPP_
