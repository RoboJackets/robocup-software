// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from rj_msgs:msg/GameState.idl
// generated code does not contain a copyright notice
#include "rj_msgs/msg/detail/game_state__rosidl_typesupport_fastrtps_cpp.hpp"
#include "rj_msgs/msg/detail/game_state__struct.hpp"

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
namespace builtin_interfaces
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const builtin_interfaces::msg::Duration &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  builtin_interfaces::msg::Duration &);
size_t get_serialized_size(
  const builtin_interfaces::msg::Duration &,
  size_t current_alignment);
size_t
max_serialized_size_Duration(
  bool & full_bounded,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace builtin_interfaces

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
  const rj_msgs::msg::GameState & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: period
  cdr << ros_message.period;
  // Member: state
  cdr << ros_message.state;
  // Member: restart
  cdr << ros_message.restart;
  // Member: our_restart
  cdr << (ros_message.our_restart ? true : false);
  // Member: stage_time_left
  builtin_interfaces::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.stage_time_left,
    cdr);
  // Member: placement_point
  rj_geometry_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.placement_point,
    cdr);
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_rj_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  rj_msgs::msg::GameState & ros_message)
{
  // Member: period
  cdr >> ros_message.period;

  // Member: state
  cdr >> ros_message.state;

  // Member: restart
  cdr >> ros_message.restart;

  // Member: our_restart
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.our_restart = tmp ? true : false;
  }

  // Member: stage_time_left
  builtin_interfaces::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.stage_time_left);

  // Member: placement_point
  rj_geometry_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.placement_point);

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_rj_msgs
get_serialized_size(
  const rj_msgs::msg::GameState & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: period
  {
    size_t item_size = sizeof(ros_message.period);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: state
  {
    size_t item_size = sizeof(ros_message.state);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: restart
  {
    size_t item_size = sizeof(ros_message.restart);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: our_restart
  {
    size_t item_size = sizeof(ros_message.our_restart);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: stage_time_left

  current_alignment +=
    builtin_interfaces::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.stage_time_left, current_alignment);
  // Member: placement_point

  current_alignment +=
    rj_geometry_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.placement_point, current_alignment);

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_rj_msgs
max_serialized_size_GameState(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;


  // Member: period
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: state
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: restart
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: our_restart
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: stage_time_left
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        builtin_interfaces::msg::typesupport_fastrtps_cpp::max_serialized_size_Duration(
        full_bounded, current_alignment);
    }
  }

  // Member: placement_point
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

static bool _GameState__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const rj_msgs::msg::GameState *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _GameState__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<rj_msgs::msg::GameState *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _GameState__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const rj_msgs::msg::GameState *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _GameState__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_GameState(full_bounded, 0);
}

static message_type_support_callbacks_t _GameState__callbacks = {
  "rj_msgs::msg",
  "GameState",
  _GameState__cdr_serialize,
  _GameState__cdr_deserialize,
  _GameState__get_serialized_size,
  _GameState__max_serialized_size
};

static rosidl_message_type_support_t _GameState__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_GameState__callbacks,
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
get_message_type_support_handle<rj_msgs::msg::GameState>()
{
  return &rj_msgs::msg::typesupport_fastrtps_cpp::_GameState__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, rj_msgs, msg, GameState)() {
  return &rj_msgs::msg::typesupport_fastrtps_cpp::_GameState__handle;
}

#ifdef __cplusplus
}
#endif
