// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from rj_msgs:msg/RobotIntent.idl
// generated code does not contain a copyright notice
#include "rj_msgs/msg/detail/robot_intent__rosidl_typesupport_fastrtps_cpp.hpp"
#include "rj_msgs/msg/detail/robot_intent__struct.hpp"

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
  const rj_msgs::msg::MotionCommand &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  rj_msgs::msg::MotionCommand &);
size_t get_serialized_size(
  const rj_msgs::msg::MotionCommand &,
  size_t current_alignment);
size_t
max_serialized_size_MotionCommand(
  bool & full_bounded,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace rj_msgs

namespace rj_geometry_msgs
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const rj_geometry_msgs::msg::ShapeSet &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  rj_geometry_msgs::msg::ShapeSet &);
size_t get_serialized_size(
  const rj_geometry_msgs::msg::ShapeSet &,
  size_t current_alignment);
size_t
max_serialized_size_ShapeSet(
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
  const rj_msgs::msg::RobotIntent & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: motion_command
  rj_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.motion_command,
    cdr);
  // Member: local_obstacles
  rj_geometry_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.local_obstacles,
    cdr);
  // Member: shoot_mode
  cdr << ros_message.shoot_mode;
  // Member: trigger_mode
  cdr << ros_message.trigger_mode;
  // Member: kick_speed
  cdr << ros_message.kick_speed;
  // Member: dribbler_speed
  cdr << ros_message.dribbler_speed;
  // Member: is_active
  cdr << (ros_message.is_active ? true : false);
  // Member: priority
  cdr << ros_message.priority;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_rj_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  rj_msgs::msg::RobotIntent & ros_message)
{
  // Member: motion_command
  rj_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.motion_command);

  // Member: local_obstacles
  rj_geometry_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.local_obstacles);

  // Member: shoot_mode
  cdr >> ros_message.shoot_mode;

  // Member: trigger_mode
  cdr >> ros_message.trigger_mode;

  // Member: kick_speed
  cdr >> ros_message.kick_speed;

  // Member: dribbler_speed
  cdr >> ros_message.dribbler_speed;

  // Member: is_active
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.is_active = tmp ? true : false;
  }

  // Member: priority
  cdr >> ros_message.priority;

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_rj_msgs
get_serialized_size(
  const rj_msgs::msg::RobotIntent & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: motion_command

  current_alignment +=
    rj_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.motion_command, current_alignment);
  // Member: local_obstacles

  current_alignment +=
    rj_geometry_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.local_obstacles, current_alignment);
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
  // Member: kick_speed
  {
    size_t item_size = sizeof(ros_message.kick_speed);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: dribbler_speed
  {
    size_t item_size = sizeof(ros_message.dribbler_speed);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: is_active
  {
    size_t item_size = sizeof(ros_message.is_active);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: priority
  {
    size_t item_size = sizeof(ros_message.priority);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_rj_msgs
max_serialized_size_RobotIntent(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;


  // Member: motion_command
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        rj_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_MotionCommand(
        full_bounded, current_alignment);
    }
  }

  // Member: local_obstacles
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        rj_geometry_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_ShapeSet(
        full_bounded, current_alignment);
    }
  }

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

  // Member: kick_speed
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: dribbler_speed
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: is_active
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: priority
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  return current_alignment - initial_alignment;
}

static bool _RobotIntent__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const rj_msgs::msg::RobotIntent *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _RobotIntent__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<rj_msgs::msg::RobotIntent *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _RobotIntent__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const rj_msgs::msg::RobotIntent *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _RobotIntent__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_RobotIntent(full_bounded, 0);
}

static message_type_support_callbacks_t _RobotIntent__callbacks = {
  "rj_msgs::msg",
  "RobotIntent",
  _RobotIntent__cdr_serialize,
  _RobotIntent__cdr_deserialize,
  _RobotIntent__get_serialized_size,
  _RobotIntent__max_serialized_size
};

static rosidl_message_type_support_t _RobotIntent__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_RobotIntent__callbacks,
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
get_message_type_support_handle<rj_msgs::msg::RobotIntent>()
{
  return &rj_msgs::msg::typesupport_fastrtps_cpp::_RobotIntent__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, rj_msgs, msg, RobotIntent)() {
  return &rj_msgs::msg::typesupport_fastrtps_cpp::_RobotIntent__handle;
}

#ifdef __cplusplus
}
#endif
