// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from rj_msgs:msg/RobotStatus.idl
// generated code does not contain a copyright notice
#include "rj_msgs/msg/detail/robot_status__rosidl_typesupport_fastrtps_cpp.hpp"
#include "rj_msgs/msg/detail/robot_status__struct.hpp"

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
  const rj_msgs::msg::RobotStatus & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: robot_id
  cdr << ros_message.robot_id;
  // Member: battery_voltage
  cdr << ros_message.battery_voltage;
  // Member: motor_errors
  {
    cdr << ros_message.motor_errors;
  }
  // Member: has_ball_sense
  cdr << (ros_message.has_ball_sense ? true : false);
  // Member: kicker_charged
  cdr << (ros_message.kicker_charged ? true : false);
  // Member: kicker_healthy
  cdr << (ros_message.kicker_healthy ? true : false);
  // Member: fpga_error
  cdr << (ros_message.fpga_error ? true : false);
  // Member: encoder_deltas
  {
    cdr << ros_message.encoder_deltas;
  }
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_rj_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  rj_msgs::msg::RobotStatus & ros_message)
{
  // Member: robot_id
  cdr >> ros_message.robot_id;

  // Member: battery_voltage
  cdr >> ros_message.battery_voltage;

  // Member: motor_errors
  {
    cdr >> ros_message.motor_errors;
  }

  // Member: has_ball_sense
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.has_ball_sense = tmp ? true : false;
  }

  // Member: kicker_charged
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.kicker_charged = tmp ? true : false;
  }

  // Member: kicker_healthy
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.kicker_healthy = tmp ? true : false;
  }

  // Member: fpga_error
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.fpga_error = tmp ? true : false;
  }

  // Member: encoder_deltas
  {
    cdr >> ros_message.encoder_deltas;
  }

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_rj_msgs
get_serialized_size(
  const rj_msgs::msg::RobotStatus & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: robot_id
  {
    size_t item_size = sizeof(ros_message.robot_id);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: battery_voltage
  {
    size_t item_size = sizeof(ros_message.battery_voltage);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: motor_errors
  {
    size_t array_size = 5;
    size_t item_size = sizeof(ros_message.motor_errors[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: has_ball_sense
  {
    size_t item_size = sizeof(ros_message.has_ball_sense);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: kicker_charged
  {
    size_t item_size = sizeof(ros_message.kicker_charged);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: kicker_healthy
  {
    size_t item_size = sizeof(ros_message.kicker_healthy);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: fpga_error
  {
    size_t item_size = sizeof(ros_message.fpga_error);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: encoder_deltas
  {
    size_t array_size = 4;
    size_t item_size = sizeof(ros_message.encoder_deltas[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_rj_msgs
max_serialized_size_RobotStatus(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;


  // Member: robot_id
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: battery_voltage
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: motor_errors
  {
    size_t array_size = 5;

    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: has_ball_sense
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: kicker_charged
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: kicker_healthy
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: fpga_error
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: encoder_deltas
  {
    size_t array_size = 4;

    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  return current_alignment - initial_alignment;
}

static bool _RobotStatus__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const rj_msgs::msg::RobotStatus *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _RobotStatus__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<rj_msgs::msg::RobotStatus *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _RobotStatus__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const rj_msgs::msg::RobotStatus *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _RobotStatus__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_RobotStatus(full_bounded, 0);
}

static message_type_support_callbacks_t _RobotStatus__callbacks = {
  "rj_msgs::msg",
  "RobotStatus",
  _RobotStatus__cdr_serialize,
  _RobotStatus__cdr_deserialize,
  _RobotStatus__get_serialized_size,
  _RobotStatus__max_serialized_size
};

static rosidl_message_type_support_t _RobotStatus__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_RobotStatus__callbacks,
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
get_message_type_support_handle<rj_msgs::msg::RobotStatus>()
{
  return &rj_msgs::msg::typesupport_fastrtps_cpp::_RobotStatus__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, rj_msgs, msg, RobotStatus)() {
  return &rj_msgs::msg::typesupport_fastrtps_cpp::_RobotStatus__handle;
}

#ifdef __cplusplus
}
#endif
