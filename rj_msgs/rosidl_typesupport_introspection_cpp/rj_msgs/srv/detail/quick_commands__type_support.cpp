// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from rj_msgs:srv/QuickCommands.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "rj_msgs/srv/detail/quick_commands__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace rj_msgs
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

void QuickCommands_Request_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) rj_msgs::srv::QuickCommands_Request(_init);
}

void QuickCommands_Request_fini_function(void * message_memory)
{
  auto typed_message = static_cast<rj_msgs::srv::QuickCommands_Request *>(message_memory);
  typed_message->~QuickCommands_Request();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember QuickCommands_Request_message_member_array[1] = {
  {
    "state",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rj_msgs::srv::QuickCommands_Request, state),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers QuickCommands_Request_message_members = {
  "rj_msgs::srv",  // message namespace
  "QuickCommands_Request",  // message name
  1,  // number of fields
  sizeof(rj_msgs::srv::QuickCommands_Request),
  QuickCommands_Request_message_member_array,  // message members
  QuickCommands_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  QuickCommands_Request_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t QuickCommands_Request_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &QuickCommands_Request_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace rj_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<rj_msgs::srv::QuickCommands_Request>()
{
  return &::rj_msgs::srv::rosidl_typesupport_introspection_cpp::QuickCommands_Request_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, rj_msgs, srv, QuickCommands_Request)() {
  return &::rj_msgs::srv::rosidl_typesupport_introspection_cpp::QuickCommands_Request_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "array"
// already included above
// #include "cstddef"
// already included above
// #include "string"
// already included above
// #include "vector"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "rj_msgs/srv/detail/quick_commands__struct.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/field_types.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace rj_msgs
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

void QuickCommands_Response_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) rj_msgs::srv::QuickCommands_Response(_init);
}

void QuickCommands_Response_fini_function(void * message_memory)
{
  auto typed_message = static_cast<rj_msgs::srv::QuickCommands_Response *>(message_memory);
  typed_message->~QuickCommands_Response();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember QuickCommands_Response_message_member_array[1] = {
  {
    "structure_needs_at_least_one_member",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rj_msgs::srv::QuickCommands_Response, structure_needs_at_least_one_member),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers QuickCommands_Response_message_members = {
  "rj_msgs::srv",  // message namespace
  "QuickCommands_Response",  // message name
  1,  // number of fields
  sizeof(rj_msgs::srv::QuickCommands_Response),
  QuickCommands_Response_message_member_array,  // message members
  QuickCommands_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  QuickCommands_Response_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t QuickCommands_Response_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &QuickCommands_Response_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace rj_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<rj_msgs::srv::QuickCommands_Response>()
{
  return &::rj_msgs::srv::rosidl_typesupport_introspection_cpp::QuickCommands_Response_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, rj_msgs, srv, QuickCommands_Response)() {
  return &::rj_msgs::srv::rosidl_typesupport_introspection_cpp::QuickCommands_Response_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "rosidl_typesupport_introspection_cpp/visibility_control.h"
// already included above
// #include "rj_msgs/srv/detail/quick_commands__struct.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/service_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/service_type_support_decl.hpp"

namespace rj_msgs
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

// this is intentionally not const to allow initialization later to prevent an initialization race
static ::rosidl_typesupport_introspection_cpp::ServiceMembers QuickCommands_service_members = {
  "rj_msgs::srv",  // service namespace
  "QuickCommands",  // service name
  // these two fields are initialized below on the first access
  // see get_service_type_support_handle<rj_msgs::srv::QuickCommands>()
  nullptr,  // request message
  nullptr  // response message
};

static const rosidl_service_type_support_t QuickCommands_service_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &QuickCommands_service_members,
  get_service_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace rj_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<rj_msgs::srv::QuickCommands>()
{
  // get a handle to the value to be returned
  auto service_type_support =
    &::rj_msgs::srv::rosidl_typesupport_introspection_cpp::QuickCommands_service_type_support_handle;
  // get a non-const and properly typed version of the data void *
  auto service_members = const_cast<::rosidl_typesupport_introspection_cpp::ServiceMembers *>(
    static_cast<const ::rosidl_typesupport_introspection_cpp::ServiceMembers *>(
      service_type_support->data));
  // make sure that both the request_members_ and the response_members_ are initialized
  // if they are not, initialize them
  if (
    service_members->request_members_ == nullptr ||
    service_members->response_members_ == nullptr)
  {
    // initialize the request_members_ with the static function from the external library
    service_members->request_members_ = static_cast<
      const ::rosidl_typesupport_introspection_cpp::MessageMembers *
      >(
      ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<
        ::rj_msgs::srv::QuickCommands_Request
      >()->data
      );
    // initialize the response_members_ with the static function from the external library
    service_members->response_members_ = static_cast<
      const ::rosidl_typesupport_introspection_cpp::MessageMembers *
      >(
      ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<
        ::rj_msgs::srv::QuickCommands_Response
      >()->data
      );
  }
  // finally return the properly initialized service_type_support handle
  return service_type_support;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, rj_msgs, srv, QuickCommands)() {
  return ::rosidl_typesupport_introspection_cpp::get_service_type_support_handle<rj_msgs::srv::QuickCommands>();
}

#ifdef __cplusplus
}
#endif
