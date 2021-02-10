// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from rj_msgs:msg/DetectionBall.idl
// generated code does not contain a copyright notice

#ifndef RJ_MSGS__MSG__DETAIL__DETECTION_BALL__STRUCT_HPP_
#define RJ_MSGS__MSG__DETAIL__DETECTION_BALL__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__rj_msgs__msg__DetectionBall __attribute__((deprecated))
#else
# define DEPRECATED__rj_msgs__msg__DetectionBall __declspec(deprecated)
#endif

namespace rj_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct DetectionBall_
{
  using Type = DetectionBall_<ContainerAllocator>;

  explicit DetectionBall_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->confidence = 0.0f;
      this->area = 0ul;
      this->x = 0.0f;
      this->y = 0.0f;
      this->z = 0.0f;
      this->pixel_x = 0.0f;
      this->pixel_y = 0.0f;
    }
  }

  explicit DetectionBall_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->confidence = 0.0f;
      this->area = 0ul;
      this->x = 0.0f;
      this->y = 0.0f;
      this->z = 0.0f;
      this->pixel_x = 0.0f;
      this->pixel_y = 0.0f;
    }
  }

  // field types and members
  using _confidence_type =
    float;
  _confidence_type confidence;
  using _area_type =
    uint32_t;
  _area_type area;
  using _x_type =
    float;
  _x_type x;
  using _y_type =
    float;
  _y_type y;
  using _z_type =
    float;
  _z_type z;
  using _pixel_x_type =
    float;
  _pixel_x_type pixel_x;
  using _pixel_y_type =
    float;
  _pixel_y_type pixel_y;

  // setters for named parameter idiom
  Type & set__confidence(
    const float & _arg)
  {
    this->confidence = _arg;
    return *this;
  }
  Type & set__area(
    const uint32_t & _arg)
  {
    this->area = _arg;
    return *this;
  }
  Type & set__x(
    const float & _arg)
  {
    this->x = _arg;
    return *this;
  }
  Type & set__y(
    const float & _arg)
  {
    this->y = _arg;
    return *this;
  }
  Type & set__z(
    const float & _arg)
  {
    this->z = _arg;
    return *this;
  }
  Type & set__pixel_x(
    const float & _arg)
  {
    this->pixel_x = _arg;
    return *this;
  }
  Type & set__pixel_y(
    const float & _arg)
  {
    this->pixel_y = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    rj_msgs::msg::DetectionBall_<ContainerAllocator> *;
  using ConstRawPtr =
    const rj_msgs::msg::DetectionBall_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rj_msgs::msg::DetectionBall_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rj_msgs::msg::DetectionBall_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rj_msgs::msg::DetectionBall_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rj_msgs::msg::DetectionBall_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rj_msgs::msg::DetectionBall_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rj_msgs::msg::DetectionBall_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rj_msgs::msg::DetectionBall_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rj_msgs::msg::DetectionBall_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rj_msgs__msg__DetectionBall
    std::shared_ptr<rj_msgs::msg::DetectionBall_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rj_msgs__msg__DetectionBall
    std::shared_ptr<rj_msgs::msg::DetectionBall_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const DetectionBall_ & other) const
  {
    if (this->confidence != other.confidence) {
      return false;
    }
    if (this->area != other.area) {
      return false;
    }
    if (this->x != other.x) {
      return false;
    }
    if (this->y != other.y) {
      return false;
    }
    if (this->z != other.z) {
      return false;
    }
    if (this->pixel_x != other.pixel_x) {
      return false;
    }
    if (this->pixel_y != other.pixel_y) {
      return false;
    }
    return true;
  }
  bool operator!=(const DetectionBall_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct DetectionBall_

// alias to use template instance with default allocator
using DetectionBall =
  rj_msgs::msg::DetectionBall_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace rj_msgs

#endif  // RJ_MSGS__MSG__DETAIL__DETECTION_BALL__STRUCT_HPP_
