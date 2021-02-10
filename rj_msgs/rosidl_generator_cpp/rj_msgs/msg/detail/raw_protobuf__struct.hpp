// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from rj_msgs:msg/RawProtobuf.idl
// generated code does not contain a copyright notice

#ifndef RJ_MSGS__MSG__DETAIL__RAW_PROTOBUF__STRUCT_HPP_
#define RJ_MSGS__MSG__DETAIL__RAW_PROTOBUF__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__rj_msgs__msg__RawProtobuf __attribute__((deprecated))
#else
# define DEPRECATED__rj_msgs__msg__RawProtobuf __declspec(deprecated)
#endif

namespace rj_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct RawProtobuf_
{
  using Type = RawProtobuf_<ContainerAllocator>;

  explicit RawProtobuf_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit RawProtobuf_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _data_type =
    std::vector<unsigned char, typename ContainerAllocator::template rebind<unsigned char>::other>;
  _data_type data;

  // setters for named parameter idiom
  Type & set__data(
    const std::vector<unsigned char, typename ContainerAllocator::template rebind<unsigned char>::other> & _arg)
  {
    this->data = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    rj_msgs::msg::RawProtobuf_<ContainerAllocator> *;
  using ConstRawPtr =
    const rj_msgs::msg::RawProtobuf_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rj_msgs::msg::RawProtobuf_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rj_msgs::msg::RawProtobuf_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rj_msgs::msg::RawProtobuf_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rj_msgs::msg::RawProtobuf_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rj_msgs::msg::RawProtobuf_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rj_msgs::msg::RawProtobuf_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rj_msgs::msg::RawProtobuf_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rj_msgs::msg::RawProtobuf_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rj_msgs__msg__RawProtobuf
    std::shared_ptr<rj_msgs::msg::RawProtobuf_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rj_msgs__msg__RawProtobuf
    std::shared_ptr<rj_msgs::msg::RawProtobuf_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RawProtobuf_ & other) const
  {
    if (this->data != other.data) {
      return false;
    }
    return true;
  }
  bool operator!=(const RawProtobuf_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RawProtobuf_

// alias to use template instance with default allocator
using RawProtobuf =
  rj_msgs::msg::RawProtobuf_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace rj_msgs

#endif  // RJ_MSGS__MSG__DETAIL__RAW_PROTOBUF__STRUCT_HPP_
