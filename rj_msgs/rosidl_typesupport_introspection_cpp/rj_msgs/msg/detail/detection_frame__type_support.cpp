// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from rj_msgs:msg/DetectionFrame.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "rj_msgs/msg/detail/detection_frame__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace rj_msgs
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void DetectionFrame_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) rj_msgs::msg::DetectionFrame(_init);
}

void DetectionFrame_fini_function(void * message_memory)
{
  auto typed_message = static_cast<rj_msgs::msg::DetectionFrame *>(message_memory);
  typed_message->~DetectionFrame();
}

size_t size_function__DetectionFrame__balls(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<rj_msgs::msg::DetectionBall> *>(untyped_member);
  return member->size();
}

const void * get_const_function__DetectionFrame__balls(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<rj_msgs::msg::DetectionBall> *>(untyped_member);
  return &member[index];
}

void * get_function__DetectionFrame__balls(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<rj_msgs::msg::DetectionBall> *>(untyped_member);
  return &member[index];
}

void resize_function__DetectionFrame__balls(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<rj_msgs::msg::DetectionBall> *>(untyped_member);
  member->resize(size);
}

size_t size_function__DetectionFrame__robots_yellow(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<rj_msgs::msg::DetectionRobot> *>(untyped_member);
  return member->size();
}

const void * get_const_function__DetectionFrame__robots_yellow(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<rj_msgs::msg::DetectionRobot> *>(untyped_member);
  return &member[index];
}

void * get_function__DetectionFrame__robots_yellow(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<rj_msgs::msg::DetectionRobot> *>(untyped_member);
  return &member[index];
}

void resize_function__DetectionFrame__robots_yellow(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<rj_msgs::msg::DetectionRobot> *>(untyped_member);
  member->resize(size);
}

size_t size_function__DetectionFrame__robots_blue(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<rj_msgs::msg::DetectionRobot> *>(untyped_member);
  return member->size();
}

const void * get_const_function__DetectionFrame__robots_blue(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<rj_msgs::msg::DetectionRobot> *>(untyped_member);
  return &member[index];
}

void * get_function__DetectionFrame__robots_blue(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<rj_msgs::msg::DetectionRobot> *>(untyped_member);
  return &member[index];
}

void resize_function__DetectionFrame__robots_blue(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<rj_msgs::msg::DetectionRobot> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember DetectionFrame_message_member_array[8] = {
  {
    "frame_number",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rj_msgs::msg::DetectionFrame, frame_number),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "t_capture",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<builtin_interfaces::msg::Time>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rj_msgs::msg::DetectionFrame, t_capture),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "t_sent",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<builtin_interfaces::msg::Time>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rj_msgs::msg::DetectionFrame, t_sent),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "t_received",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<builtin_interfaces::msg::Time>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rj_msgs::msg::DetectionFrame, t_received),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "camera_id",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rj_msgs::msg::DetectionFrame, camera_id),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "balls",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<rj_msgs::msg::DetectionBall>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rj_msgs::msg::DetectionFrame, balls),  // bytes offset in struct
    nullptr,  // default value
    size_function__DetectionFrame__balls,  // size() function pointer
    get_const_function__DetectionFrame__balls,  // get_const(index) function pointer
    get_function__DetectionFrame__balls,  // get(index) function pointer
    resize_function__DetectionFrame__balls  // resize(index) function pointer
  },
  {
    "robots_yellow",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<rj_msgs::msg::DetectionRobot>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rj_msgs::msg::DetectionFrame, robots_yellow),  // bytes offset in struct
    nullptr,  // default value
    size_function__DetectionFrame__robots_yellow,  // size() function pointer
    get_const_function__DetectionFrame__robots_yellow,  // get_const(index) function pointer
    get_function__DetectionFrame__robots_yellow,  // get(index) function pointer
    resize_function__DetectionFrame__robots_yellow  // resize(index) function pointer
  },
  {
    "robots_blue",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<rj_msgs::msg::DetectionRobot>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rj_msgs::msg::DetectionFrame, robots_blue),  // bytes offset in struct
    nullptr,  // default value
    size_function__DetectionFrame__robots_blue,  // size() function pointer
    get_const_function__DetectionFrame__robots_blue,  // get_const(index) function pointer
    get_function__DetectionFrame__robots_blue,  // get(index) function pointer
    resize_function__DetectionFrame__robots_blue  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers DetectionFrame_message_members = {
  "rj_msgs::msg",  // message namespace
  "DetectionFrame",  // message name
  8,  // number of fields
  sizeof(rj_msgs::msg::DetectionFrame),
  DetectionFrame_message_member_array,  // message members
  DetectionFrame_init_function,  // function to initialize message memory (memory has to be allocated)
  DetectionFrame_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t DetectionFrame_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &DetectionFrame_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace rj_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<rj_msgs::msg::DetectionFrame>()
{
  return &::rj_msgs::msg::rosidl_typesupport_introspection_cpp::DetectionFrame_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, rj_msgs, msg, DetectionFrame)() {
  return &::rj_msgs::msg::rosidl_typesupport_introspection_cpp::DetectionFrame_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
