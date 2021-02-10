// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from rj_msgs:msg/FieldDimensions.idl
// generated code does not contain a copyright notice

#ifndef RJ_MSGS__MSG__DETAIL__FIELD_DIMENSIONS__STRUCT_HPP_
#define RJ_MSGS__MSG__DETAIL__FIELD_DIMENSIONS__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__rj_msgs__msg__FieldDimensions __attribute__((deprecated))
#else
# define DEPRECATED__rj_msgs__msg__FieldDimensions __declspec(deprecated)
#endif

namespace rj_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct FieldDimensions_
{
  using Type = FieldDimensions_<ContainerAllocator>;

  explicit FieldDimensions_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->length = 0.0f;
      this->width = 0.0f;
      this->border = 0.0f;
      this->line_width = 0.0f;
      this->goal_width = 0.0f;
      this->goal_depth = 0.0f;
      this->goal_height = 0.0f;
      this->penalty_short_dist = 0.0f;
      this->penalty_long_dist = 0.0f;
      this->center_radius = 0.0f;
      this->center_diameter = 0.0f;
      this->goal_flat = 0.0f;
      this->floor_length = 0.0f;
      this->floor_width = 0.0f;
    }
  }

  explicit FieldDimensions_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->length = 0.0f;
      this->width = 0.0f;
      this->border = 0.0f;
      this->line_width = 0.0f;
      this->goal_width = 0.0f;
      this->goal_depth = 0.0f;
      this->goal_height = 0.0f;
      this->penalty_short_dist = 0.0f;
      this->penalty_long_dist = 0.0f;
      this->center_radius = 0.0f;
      this->center_diameter = 0.0f;
      this->goal_flat = 0.0f;
      this->floor_length = 0.0f;
      this->floor_width = 0.0f;
    }
  }

  // field types and members
  using _length_type =
    float;
  _length_type length;
  using _width_type =
    float;
  _width_type width;
  using _border_type =
    float;
  _border_type border;
  using _line_width_type =
    float;
  _line_width_type line_width;
  using _goal_width_type =
    float;
  _goal_width_type goal_width;
  using _goal_depth_type =
    float;
  _goal_depth_type goal_depth;
  using _goal_height_type =
    float;
  _goal_height_type goal_height;
  using _penalty_short_dist_type =
    float;
  _penalty_short_dist_type penalty_short_dist;
  using _penalty_long_dist_type =
    float;
  _penalty_long_dist_type penalty_long_dist;
  using _center_radius_type =
    float;
  _center_radius_type center_radius;
  using _center_diameter_type =
    float;
  _center_diameter_type center_diameter;
  using _goal_flat_type =
    float;
  _goal_flat_type goal_flat;
  using _floor_length_type =
    float;
  _floor_length_type floor_length;
  using _floor_width_type =
    float;
  _floor_width_type floor_width;

  // setters for named parameter idiom
  Type & set__length(
    const float & _arg)
  {
    this->length = _arg;
    return *this;
  }
  Type & set__width(
    const float & _arg)
  {
    this->width = _arg;
    return *this;
  }
  Type & set__border(
    const float & _arg)
  {
    this->border = _arg;
    return *this;
  }
  Type & set__line_width(
    const float & _arg)
  {
    this->line_width = _arg;
    return *this;
  }
  Type & set__goal_width(
    const float & _arg)
  {
    this->goal_width = _arg;
    return *this;
  }
  Type & set__goal_depth(
    const float & _arg)
  {
    this->goal_depth = _arg;
    return *this;
  }
  Type & set__goal_height(
    const float & _arg)
  {
    this->goal_height = _arg;
    return *this;
  }
  Type & set__penalty_short_dist(
    const float & _arg)
  {
    this->penalty_short_dist = _arg;
    return *this;
  }
  Type & set__penalty_long_dist(
    const float & _arg)
  {
    this->penalty_long_dist = _arg;
    return *this;
  }
  Type & set__center_radius(
    const float & _arg)
  {
    this->center_radius = _arg;
    return *this;
  }
  Type & set__center_diameter(
    const float & _arg)
  {
    this->center_diameter = _arg;
    return *this;
  }
  Type & set__goal_flat(
    const float & _arg)
  {
    this->goal_flat = _arg;
    return *this;
  }
  Type & set__floor_length(
    const float & _arg)
  {
    this->floor_length = _arg;
    return *this;
  }
  Type & set__floor_width(
    const float & _arg)
  {
    this->floor_width = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    rj_msgs::msg::FieldDimensions_<ContainerAllocator> *;
  using ConstRawPtr =
    const rj_msgs::msg::FieldDimensions_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rj_msgs::msg::FieldDimensions_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rj_msgs::msg::FieldDimensions_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rj_msgs::msg::FieldDimensions_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rj_msgs::msg::FieldDimensions_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rj_msgs::msg::FieldDimensions_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rj_msgs::msg::FieldDimensions_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rj_msgs::msg::FieldDimensions_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rj_msgs::msg::FieldDimensions_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rj_msgs__msg__FieldDimensions
    std::shared_ptr<rj_msgs::msg::FieldDimensions_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rj_msgs__msg__FieldDimensions
    std::shared_ptr<rj_msgs::msg::FieldDimensions_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const FieldDimensions_ & other) const
  {
    if (this->length != other.length) {
      return false;
    }
    if (this->width != other.width) {
      return false;
    }
    if (this->border != other.border) {
      return false;
    }
    if (this->line_width != other.line_width) {
      return false;
    }
    if (this->goal_width != other.goal_width) {
      return false;
    }
    if (this->goal_depth != other.goal_depth) {
      return false;
    }
    if (this->goal_height != other.goal_height) {
      return false;
    }
    if (this->penalty_short_dist != other.penalty_short_dist) {
      return false;
    }
    if (this->penalty_long_dist != other.penalty_long_dist) {
      return false;
    }
    if (this->center_radius != other.center_radius) {
      return false;
    }
    if (this->center_diameter != other.center_diameter) {
      return false;
    }
    if (this->goal_flat != other.goal_flat) {
      return false;
    }
    if (this->floor_length != other.floor_length) {
      return false;
    }
    if (this->floor_width != other.floor_width) {
      return false;
    }
    return true;
  }
  bool operator!=(const FieldDimensions_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct FieldDimensions_

// alias to use template instance with default allocator
using FieldDimensions =
  rj_msgs::msg::FieldDimensions_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace rj_msgs

#endif  // RJ_MSGS__MSG__DETAIL__FIELD_DIMENSIONS__STRUCT_HPP_
