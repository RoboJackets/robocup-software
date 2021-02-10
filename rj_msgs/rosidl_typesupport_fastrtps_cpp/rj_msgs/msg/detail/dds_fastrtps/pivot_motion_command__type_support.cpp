// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from rj_msgs:msg/PivotMotionCommand.idl
// generated code does not contain a copyright notice
#include "rj_msgs/msg/detail/pivot_motion_command__rosidl_typesupport_fastrtps_cpp.hpp"
#include "rj_msgs/msg/detail/pivot_motion_command__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions
namespace rj_geometry_msgs
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const rj_geometry_msgs::msg::Point &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  rj_geometry_msgs::msg::Point &);
size_t get_serialized_size(
  const rj_geometry_msgs::msg::Point &,
  size_t current_alignment);
size_t
max_serialized_size_Point(
  bool & full_bounded,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace rj_geometry_msgs

namespace rj_geometry_msgs
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const rj_geometry_msgs::msg::Point &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  rj_geometry_msgs::msg::Point &);
size_t get_serialized_size(
  const rj_geometry_msgs::msg::Point &,
  size_t current_alignment);
size_t
max_serialized_size_Point(
  bool & full_bounded,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace rj_geometry_msgs


namespace rj_msgs
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_rj_msgs
cdr_serialize(
  const rj_msgs::msg::PivotMotionCommand & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: pivot_point
  rj_geometry_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.pivot_point,
    cdr);
  // Member: pivot_target
  rj_geometry_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.pivot_target,
    cdr);
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_rj_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  rj_msgs::msg::PivotMotionCommand & ros_message)
{
  // Member: pivot_point
  rj_geometry_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.pivot_point);

  // Member: pivot_target
  rj_geometry_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.pivot_target);

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_rj_msgs
get_serialized_size(
  const rj_msgs::msg::PivotMotionCommand & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: pivot_point

  current_alignment +=
    rj_geometry_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.pivot_point, current_alignment);
  // Member: pivot_target

  current_alignment +=
    rj_geometry_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.pivot_target, current_alignment);

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_rj_msgs
max_serialized_size_PivotMotionCommand(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;


  // Member: pivot_point
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        rj_geometry_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_Point(
        full_bounded, current_alignment);
    }
  }

  // Member: pivot_target
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        rj_geometry_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_Point(
        full_bounded, current_alignment);
    }
  }

  return current_alignment - initial_alignment;
}

static bool _PivotMotionCommand__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const rj_msgs::msg::PivotMotionCommand *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _PivotMotionCommand__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<rj_msgs::msg::PivotMotionCommand *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _PivotMotionCommand__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const rj_msgs::msg::PivotMotionCommand *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _PivotMotionCommand__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_PivotMotionCommand(full_bounded, 0);
}

static message_type_support_callbacks_t _PivotMotionCommand__callbacks = {
  "rj_msgs::msg",
  "PivotMotionCommand",
  _PivotMotionCommand__cdr_serialize,
  _PivotMotionCommand__cdr_deserialize,
  _PivotMotionCommand__get_serialized_size,
  _PivotMotionCommand__max_serialized_size
};

static rosidl_message_type_support_t _PivotMotionCommand__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_PivotMotionCommand__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace rj_msgs

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_rj_msgs
const rosidl_message_type_support_t *
get_message_type_support_handle<rj_msgs::msg::PivotMotionCommand>()
{
  return &rj_msgs::msg::typesupport_fastrtps_cpp::_PivotMotionCommand__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, rj_msgs, msg, PivotMotionCommand)() {
  return &rj_msgs::msg::typesupport_fastrtps_cpp::_PivotMotionCommand__handle;
}

#ifdef __cplusplus
}
#endif
