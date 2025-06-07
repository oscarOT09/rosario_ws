// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from rosario_path:msg/RosarioPath.idl
// generated code does not contain a copyright notice

#ifndef ROSARIO_PATH__MSG__DETAIL__ROSARIO_PATH__STRUCT_HPP_
#define ROSARIO_PATH__MSG__DETAIL__ROSARIO_PATH__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__rosario_path__msg__RosarioPath __attribute__((deprecated))
#else
# define DEPRECATED__rosario_path__msg__RosarioPath __declspec(deprecated)
#endif

namespace rosario_path
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct RosarioPath_
{
  using Type = RosarioPath_<ContainerAllocator>;

  explicit RosarioPath_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->error = 0.0f;
      this->curva = false;
    }
  }

  explicit RosarioPath_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->error = 0.0f;
      this->curva = false;
    }
  }

  // field types and members
  using _error_type =
    float;
  _error_type error;
  using _curva_type =
    bool;
  _curva_type curva;

  // setters for named parameter idiom
  Type & set__error(
    const float & _arg)
  {
    this->error = _arg;
    return *this;
  }
  Type & set__curva(
    const bool & _arg)
  {
    this->curva = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    rosario_path::msg::RosarioPath_<ContainerAllocator> *;
  using ConstRawPtr =
    const rosario_path::msg::RosarioPath_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rosario_path::msg::RosarioPath_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rosario_path::msg::RosarioPath_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rosario_path::msg::RosarioPath_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rosario_path::msg::RosarioPath_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rosario_path::msg::RosarioPath_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rosario_path::msg::RosarioPath_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rosario_path::msg::RosarioPath_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rosario_path::msg::RosarioPath_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rosario_path__msg__RosarioPath
    std::shared_ptr<rosario_path::msg::RosarioPath_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rosario_path__msg__RosarioPath
    std::shared_ptr<rosario_path::msg::RosarioPath_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RosarioPath_ & other) const
  {
    if (this->error != other.error) {
      return false;
    }
    if (this->curva != other.curva) {
      return false;
    }
    return true;
  }
  bool operator!=(const RosarioPath_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RosarioPath_

// alias to use template instance with default allocator
using RosarioPath =
  rosario_path::msg::RosarioPath_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace rosario_path

#endif  // ROSARIO_PATH__MSG__DETAIL__ROSARIO_PATH__STRUCT_HPP_
