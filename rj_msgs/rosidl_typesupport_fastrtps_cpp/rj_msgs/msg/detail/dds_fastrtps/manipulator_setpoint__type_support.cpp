// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from rj_msgs:msg/ManipulatorSetpoint.idl
// generated code does not contain a copyright notice
#include "rj_msgs/msg/detail/manipulator_setpoint__rosidl_typesupport_fastrtps_cpp.hpp"
#include "rj_msgs/msg/detail/manipulator_setpoint__struct.hpp"

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

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_rj_msgs
cdr_serialize(
  const rj_msgs::msg::ManipulatorSetpoint & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: shoot_mode
  cdr << ros_message.shoot_mode;
  // Member: trigger_mode
  cdr << ros_message.trigger_mode;
  // Member: kick_strength
  cdr << ros_message.kick_strength;
  // Member: dribbler_speed
  cdr << ros_message.dribbler_speed;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_rj_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  rj_msgs::msg::ManipulatorSetpoint & ros_message)
{
  // Member: shoot_mode
  cdr >> ros_message.shoot_mode;

  // Member: trigger_mode
  cdr >> ros_message.trigger_mode;

  // Member: kick_strength
  cdr >> ros_message.kick_strength;

  // Member: dribbler_speed
  cdr >> ros_message.dribbler_speed;

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_rj_msgs
get_serialized_size(
  const rj_msgs::msg::ManipulatorSetpoint & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: shoot_mode
  {
    size_t item_size = sizeof(ros_message.shoot_mode);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: trigger_mode
  {
    size_t item_size = sizeof(ros_message.trigger_mode);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: kick_strength
  {
    size_t item_size = sizeof(ros_message.kick_strength);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: dribbler_speed
  {
    size_t item_size = sizeof(ros_message.dribbler_speed);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_rj_msgs
max_serialized_size_ManipulatorSetpoint(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;


  // Member: shoot_mode
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: trigger_mode
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: kick_strength
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: dribbler_speed
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  return current_alignment - initial_alignment;
}

static bool _ManipulatorSetpoint__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const rj_msgs::msg::ManipulatorSetpoint *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _ManipulatorSetpoint__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<rj_msgs::msg::ManipulatorSetpoint *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _ManipulatorSetpoint__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const rj_msgs::msg::ManipulatorSetpoint *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _ManipulatorSetpoint__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_ManipulatorSetpoint(full_bounded, 0);
}

static message_type_support_callbacks_t _ManipulatorSetpoint__callbacks = {
  "rj_msgs::msg",
  "ManipulatorSetpoint",
  _ManipulatorSetpoint__cdr_serialize,
  _ManipulatorSetpoint__cdr_deserialize,
  _ManipulatorSetpoint__get_serialized_size,
  _ManipulatorSetpoint__max_serialized_size
};

static rosidl_message_type_support_t _ManipulatorSetpoint__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_ManipulatorSetpoint__callbacks,
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
get_message_type_support_handle<rj_msgs::msg::ManipulatorSetpoint>()
{
  return &rj_msgs::msg::typesupport_fastrtps_cpp::_ManipulatorSetpoint__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, rj_msgs, msg, ManipulatorSetpoint)() {
  return &rj_msgs::msg::typesupport_fastrtps_cpp::_ManipulatorSetpoint__handle;
}

#ifdef __cplusplus
}
#endif
