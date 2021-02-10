// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from rj_msgs:msg/MotionCommand.idl
// generated code does not contain a copyright notice
#include "rj_msgs/msg/detail/motion_command__rosidl_typesupport_fastrtps_cpp.hpp"
#include "rj_msgs/msg/detail/motion_command__struct.hpp"

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
namespace rj_msgs
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const rj_msgs::msg::EmptyMotionCommand &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  rj_msgs::msg::EmptyMotionCommand &);
size_t get_serialized_size(
  const rj_msgs::msg::EmptyMotionCommand &,
  size_t current_alignment);
size_t
max_serialized_size_EmptyMotionCommand(
  bool & full_bounded,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace rj_msgs

namespace rj_msgs
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const rj_msgs::msg::PathTargetMotionCommand &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  rj_msgs::msg::PathTargetMotionCommand &);
size_t get_serialized_size(
  const rj_msgs::msg::PathTargetMotionCommand &,
  size_t current_alignment);
size_t
max_serialized_size_PathTargetMotionCommand(
  bool & full_bounded,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace rj_msgs

namespace rj_msgs
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const rj_msgs::msg::WorldVelMotionCommand &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  rj_msgs::msg::WorldVelMotionCommand &);
size_t get_serialized_size(
  const rj_msgs::msg::WorldVelMotionCommand &,
  size_t current_alignment);
size_t
max_serialized_size_WorldVelMotionCommand(
  bool & full_bounded,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace rj_msgs

namespace rj_msgs
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const rj_msgs::msg::PivotMotionCommand &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  rj_msgs::msg::PivotMotionCommand &);
size_t get_serialized_size(
  const rj_msgs::msg::PivotMotionCommand &,
  size_t current_alignment);
size_t
max_serialized_size_PivotMotionCommand(
  bool & full_bounded,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace rj_msgs

namespace rj_msgs
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const rj_msgs::msg::SettleMotionCommand &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  rj_msgs::msg::SettleMotionCommand &);
size_t get_serialized_size(
  const rj_msgs::msg::SettleMotionCommand &,
  size_t current_alignment);
size_t
max_serialized_size_SettleMotionCommand(
  bool & full_bounded,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace rj_msgs

namespace rj_msgs
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const rj_msgs::msg::CollectMotionCommand &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  rj_msgs::msg::CollectMotionCommand &);
size_t get_serialized_size(
  const rj_msgs::msg::CollectMotionCommand &,
  size_t current_alignment);
size_t
max_serialized_size_CollectMotionCommand(
  bool & full_bounded,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace rj_msgs

namespace rj_msgs
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const rj_msgs::msg::LineKickMotionCommand &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  rj_msgs::msg::LineKickMotionCommand &);
size_t get_serialized_size(
  const rj_msgs::msg::LineKickMotionCommand &,
  size_t current_alignment);
size_t
max_serialized_size_LineKickMotionCommand(
  bool & full_bounded,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace rj_msgs

namespace rj_msgs
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const rj_msgs::msg::InterceptMotionCommand &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  rj_msgs::msg::InterceptMotionCommand &);
size_t get_serialized_size(
  const rj_msgs::msg::InterceptMotionCommand &,
  size_t current_alignment);
size_t
max_serialized_size_InterceptMotionCommand(
  bool & full_bounded,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace rj_msgs


namespace rj_msgs
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_rj_msgs
cdr_serialize(
  const rj_msgs::msg::MotionCommand & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: empty_command
  {
    size_t size = ros_message.empty_command.size();
    if (size > 1) {
      throw std::runtime_error("array size exceeds upper bound");
    }
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; i++) {
      rj_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
        ros_message.empty_command[i],
        cdr);
    }
  }
  // Member: path_target_command
  {
    size_t size = ros_message.path_target_command.size();
    if (size > 1) {
      throw std::runtime_error("array size exceeds upper bound");
    }
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; i++) {
      rj_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
        ros_message.path_target_command[i],
        cdr);
    }
  }
  // Member: world_vel_command
  {
    size_t size = ros_message.world_vel_command.size();
    if (size > 1) {
      throw std::runtime_error("array size exceeds upper bound");
    }
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; i++) {
      rj_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
        ros_message.world_vel_command[i],
        cdr);
    }
  }
  // Member: pivot_command
  {
    size_t size = ros_message.pivot_command.size();
    if (size > 1) {
      throw std::runtime_error("array size exceeds upper bound");
    }
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; i++) {
      rj_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
        ros_message.pivot_command[i],
        cdr);
    }
  }
  // Member: settle_command
  {
    size_t size = ros_message.settle_command.size();
    if (size > 1) {
      throw std::runtime_error("array size exceeds upper bound");
    }
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; i++) {
      rj_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
        ros_message.settle_command[i],
        cdr);
    }
  }
  // Member: collect_command
  {
    size_t size = ros_message.collect_command.size();
    if (size > 1) {
      throw std::runtime_error("array size exceeds upper bound");
    }
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; i++) {
      rj_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
        ros_message.collect_command[i],
        cdr);
    }
  }
  // Member: line_kick_command
  {
    size_t size = ros_message.line_kick_command.size();
    if (size > 1) {
      throw std::runtime_error("array size exceeds upper bound");
    }
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; i++) {
      rj_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
        ros_message.line_kick_command[i],
        cdr);
    }
  }
  // Member: intercept_command
  {
    size_t size = ros_message.intercept_command.size();
    if (size > 1) {
      throw std::runtime_error("array size exceeds upper bound");
    }
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; i++) {
      rj_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
        ros_message.intercept_command[i],
        cdr);
    }
  }
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_rj_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  rj_msgs::msg::MotionCommand & ros_message)
{
  // Member: empty_command
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    ros_message.empty_command.resize(size);
    for (size_t i = 0; i < size; i++) {
      rj_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
        cdr, ros_message.empty_command[i]);
    }
  }

  // Member: path_target_command
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    ros_message.path_target_command.resize(size);
    for (size_t i = 0; i < size; i++) {
      rj_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
        cdr, ros_message.path_target_command[i]);
    }
  }

  // Member: world_vel_command
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    ros_message.world_vel_command.resize(size);
    for (size_t i = 0; i < size; i++) {
      rj_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
        cdr, ros_message.world_vel_command[i]);
    }
  }

  // Member: pivot_command
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    ros_message.pivot_command.resize(size);
    for (size_t i = 0; i < size; i++) {
      rj_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
        cdr, ros_message.pivot_command[i]);
    }
  }

  // Member: settle_command
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    ros_message.settle_command.resize(size);
    for (size_t i = 0; i < size; i++) {
      rj_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
        cdr, ros_message.settle_command[i]);
    }
  }

  // Member: collect_command
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    ros_message.collect_command.resize(size);
    for (size_t i = 0; i < size; i++) {
      rj_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
        cdr, ros_message.collect_command[i]);
    }
  }

  // Member: line_kick_command
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    ros_message.line_kick_command.resize(size);
    for (size_t i = 0; i < size; i++) {
      rj_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
        cdr, ros_message.line_kick_command[i]);
    }
  }

  // Member: intercept_command
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    ros_message.intercept_command.resize(size);
    for (size_t i = 0; i < size; i++) {
      rj_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
        cdr, ros_message.intercept_command[i]);
    }
  }

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_rj_msgs
get_serialized_size(
  const rj_msgs::msg::MotionCommand & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: empty_command
  {
    size_t array_size = ros_message.empty_command.size();
    if (array_size > 1) {
      throw std::runtime_error("array size exceeds upper bound");
    }

    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        rj_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
        ros_message.empty_command[index], current_alignment);
    }
  }
  // Member: path_target_command
  {
    size_t array_size = ros_message.path_target_command.size();
    if (array_size > 1) {
      throw std::runtime_error("array size exceeds upper bound");
    }

    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        rj_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
        ros_message.path_target_command[index], current_alignment);
    }
  }
  // Member: world_vel_command
  {
    size_t array_size = ros_message.world_vel_command.size();
    if (array_size > 1) {
      throw std::runtime_error("array size exceeds upper bound");
    }

    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        rj_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
        ros_message.world_vel_command[index], current_alignment);
    }
  }
  // Member: pivot_command
  {
    size_t array_size = ros_message.pivot_command.size();
    if (array_size > 1) {
      throw std::runtime_error("array size exceeds upper bound");
    }

    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        rj_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
        ros_message.pivot_command[index], current_alignment);
    }
  }
  // Member: settle_command
  {
    size_t array_size = ros_message.settle_command.size();
    if (array_size > 1) {
      throw std::runtime_error("array size exceeds upper bound");
    }

    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        rj_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
        ros_message.settle_command[index], current_alignment);
    }
  }
  // Member: collect_command
  {
    size_t array_size = ros_message.collect_command.size();
    if (array_size > 1) {
      throw std::runtime_error("array size exceeds upper bound");
    }

    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        rj_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
        ros_message.collect_command[index], current_alignment);
    }
  }
  // Member: line_kick_command
  {
    size_t array_size = ros_message.line_kick_command.size();
    if (array_size > 1) {
      throw std::runtime_error("array size exceeds upper bound");
    }

    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        rj_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
        ros_message.line_kick_command[index], current_alignment);
    }
  }
  // Member: intercept_command
  {
    size_t array_size = ros_message.intercept_command.size();
    if (array_size > 1) {
      throw std::runtime_error("array size exceeds upper bound");
    }

    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        rj_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
        ros_message.intercept_command[index], current_alignment);
    }
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_rj_msgs
max_serialized_size_MotionCommand(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;


  // Member: empty_command
  {
    size_t array_size = 1;
    full_bounded = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        rj_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_EmptyMotionCommand(
        full_bounded, current_alignment);
    }
  }

  // Member: path_target_command
  {
    size_t array_size = 1;
    full_bounded = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        rj_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_PathTargetMotionCommand(
        full_bounded, current_alignment);
    }
  }

  // Member: world_vel_command
  {
    size_t array_size = 1;
    full_bounded = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        rj_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_WorldVelMotionCommand(
        full_bounded, current_alignment);
    }
  }

  // Member: pivot_command
  {
    size_t array_size = 1;
    full_bounded = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        rj_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_PivotMotionCommand(
        full_bounded, current_alignment);
    }
  }

  // Member: settle_command
  {
    size_t array_size = 1;
    full_bounded = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        rj_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_SettleMotionCommand(
        full_bounded, current_alignment);
    }
  }

  // Member: collect_command
  {
    size_t array_size = 1;
    full_bounded = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        rj_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_CollectMotionCommand(
        full_bounded, current_alignment);
    }
  }

  // Member: line_kick_command
  {
    size_t array_size = 1;
    full_bounded = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        rj_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_LineKickMotionCommand(
        full_bounded, current_alignment);
    }
  }

  // Member: intercept_command
  {
    size_t array_size = 1;
    full_bounded = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        rj_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_InterceptMotionCommand(
        full_bounded, current_alignment);
    }
  }

  return current_alignment - initial_alignment;
}

static bool _MotionCommand__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const rj_msgs::msg::MotionCommand *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _MotionCommand__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<rj_msgs::msg::MotionCommand *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _MotionCommand__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const rj_msgs::msg::MotionCommand *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _MotionCommand__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_MotionCommand(full_bounded, 0);
}

static message_type_support_callbacks_t _MotionCommand__callbacks = {
  "rj_msgs::msg",
  "MotionCommand",
  _MotionCommand__cdr_serialize,
  _MotionCommand__cdr_deserialize,
  _MotionCommand__get_serialized_size,
  _MotionCommand__max_serialized_size
};

static rosidl_message_type_support_t _MotionCommand__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_MotionCommand__callbacks,
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
get_message_type_support_handle<rj_msgs::msg::MotionCommand>()
{
  return &rj_msgs::msg::typesupport_fastrtps_cpp::_MotionCommand__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, rj_msgs, msg, MotionCommand)() {
  return &rj_msgs::msg::typesupport_fastrtps_cpp::_MotionCommand__handle;
}

#ifdef __cplusplus
}
#endif
