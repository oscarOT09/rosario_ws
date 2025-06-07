// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from feedback_msg:msg/YoloResults.idl
// generated code does not contain a copyright notice

#ifndef FEEDBACK_MSG__MSG__DETAIL__YOLO_RESULTS__STRUCT_HPP_
#define FEEDBACK_MSG__MSG__DETAIL__YOLO_RESULTS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__feedback_msg__msg__YoloResults __attribute__((deprecated))
#else
# define DEPRECATED__feedback_msg__msg__YoloResults __declspec(deprecated)
#endif

namespace feedback_msg
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct YoloResults_
{
  using Type = YoloResults_<ContainerAllocator>;

  explicit YoloResults_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->dotted_line = false;
      this->signal = 0l;
      this->traffic_light = 0l;
    }
  }

  explicit YoloResults_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->dotted_line = false;
      this->signal = 0l;
      this->traffic_light = 0l;
    }
  }

  // field types and members
  using _dotted_line_type =
    bool;
  _dotted_line_type dotted_line;
  using _signal_type =
    int32_t;
  _signal_type signal;
  using _traffic_light_type =
    int32_t;
  _traffic_light_type traffic_light;

  // setters for named parameter idiom
  Type & set__dotted_line(
    const bool & _arg)
  {
    this->dotted_line = _arg;
    return *this;
  }
  Type & set__signal(
    const int32_t & _arg)
  {
    this->signal = _arg;
    return *this;
  }
  Type & set__traffic_light(
    const int32_t & _arg)
  {
    this->traffic_light = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    feedback_msg::msg::YoloResults_<ContainerAllocator> *;
  using ConstRawPtr =
    const feedback_msg::msg::YoloResults_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<feedback_msg::msg::YoloResults_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<feedback_msg::msg::YoloResults_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      feedback_msg::msg::YoloResults_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<feedback_msg::msg::YoloResults_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      feedback_msg::msg::YoloResults_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<feedback_msg::msg::YoloResults_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<feedback_msg::msg::YoloResults_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<feedback_msg::msg::YoloResults_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__feedback_msg__msg__YoloResults
    std::shared_ptr<feedback_msg::msg::YoloResults_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__feedback_msg__msg__YoloResults
    std::shared_ptr<feedback_msg::msg::YoloResults_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const YoloResults_ & other) const
  {
    if (this->dotted_line != other.dotted_line) {
      return false;
    }
    if (this->signal != other.signal) {
      return false;
    }
    if (this->traffic_light != other.traffic_light) {
      return false;
    }
    return true;
  }
  bool operator!=(const YoloResults_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct YoloResults_

// alias to use template instance with default allocator
using YoloResults =
  feedback_msg::msg::YoloResults_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace feedback_msg

#endif  // FEEDBACK_MSG__MSG__DETAIL__YOLO_RESULTS__STRUCT_HPP_
