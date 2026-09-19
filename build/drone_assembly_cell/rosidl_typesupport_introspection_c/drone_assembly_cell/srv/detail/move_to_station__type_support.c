// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from drone_assembly_cell:srv/MoveToStation.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "drone_assembly_cell/srv/detail/move_to_station__rosidl_typesupport_introspection_c.h"
#include "drone_assembly_cell/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "drone_assembly_cell/srv/detail/move_to_station__functions.h"
#include "drone_assembly_cell/srv/detail/move_to_station__struct.h"


// Include directives for member types
// Member `drone_id`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void drone_assembly_cell__srv__MoveToStation_Request__rosidl_typesupport_introspection_c__MoveToStation_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  drone_assembly_cell__srv__MoveToStation_Request__init(message_memory);
}

void drone_assembly_cell__srv__MoveToStation_Request__rosidl_typesupport_introspection_c__MoveToStation_Request_fini_function(void * message_memory)
{
  drone_assembly_cell__srv__MoveToStation_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember drone_assembly_cell__srv__MoveToStation_Request__rosidl_typesupport_introspection_c__MoveToStation_Request_message_member_array[2] = {
  {
    "drone_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(drone_assembly_cell__srv__MoveToStation_Request, drone_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "target_station",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(drone_assembly_cell__srv__MoveToStation_Request, target_station),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers drone_assembly_cell__srv__MoveToStation_Request__rosidl_typesupport_introspection_c__MoveToStation_Request_message_members = {
  "drone_assembly_cell__srv",  // message namespace
  "MoveToStation_Request",  // message name
  2,  // number of fields
  sizeof(drone_assembly_cell__srv__MoveToStation_Request),
  false,  // has_any_key_member_
  drone_assembly_cell__srv__MoveToStation_Request__rosidl_typesupport_introspection_c__MoveToStation_Request_message_member_array,  // message members
  drone_assembly_cell__srv__MoveToStation_Request__rosidl_typesupport_introspection_c__MoveToStation_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  drone_assembly_cell__srv__MoveToStation_Request__rosidl_typesupport_introspection_c__MoveToStation_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t drone_assembly_cell__srv__MoveToStation_Request__rosidl_typesupport_introspection_c__MoveToStation_Request_message_type_support_handle = {
  0,
  &drone_assembly_cell__srv__MoveToStation_Request__rosidl_typesupport_introspection_c__MoveToStation_Request_message_members,
  get_message_typesupport_handle_function,
  &drone_assembly_cell__srv__MoveToStation_Request__get_type_hash,
  &drone_assembly_cell__srv__MoveToStation_Request__get_type_description,
  &drone_assembly_cell__srv__MoveToStation_Request__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_drone_assembly_cell
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, drone_assembly_cell, srv, MoveToStation_Request)() {
  if (!drone_assembly_cell__srv__MoveToStation_Request__rosidl_typesupport_introspection_c__MoveToStation_Request_message_type_support_handle.typesupport_identifier) {
    drone_assembly_cell__srv__MoveToStation_Request__rosidl_typesupport_introspection_c__MoveToStation_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &drone_assembly_cell__srv__MoveToStation_Request__rosidl_typesupport_introspection_c__MoveToStation_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "drone_assembly_cell/srv/detail/move_to_station__rosidl_typesupport_introspection_c.h"
// already included above
// #include "drone_assembly_cell/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "drone_assembly_cell/srv/detail/move_to_station__functions.h"
// already included above
// #include "drone_assembly_cell/srv/detail/move_to_station__struct.h"


// Include directives for member types
// Member `message`
// already included above
// #include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void drone_assembly_cell__srv__MoveToStation_Response__rosidl_typesupport_introspection_c__MoveToStation_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  drone_assembly_cell__srv__MoveToStation_Response__init(message_memory);
}

void drone_assembly_cell__srv__MoveToStation_Response__rosidl_typesupport_introspection_c__MoveToStation_Response_fini_function(void * message_memory)
{
  drone_assembly_cell__srv__MoveToStation_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember drone_assembly_cell__srv__MoveToStation_Response__rosidl_typesupport_introspection_c__MoveToStation_Response_message_member_array[2] = {
  {
    "success",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(drone_assembly_cell__srv__MoveToStation_Response, success),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "message",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(drone_assembly_cell__srv__MoveToStation_Response, message),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers drone_assembly_cell__srv__MoveToStation_Response__rosidl_typesupport_introspection_c__MoveToStation_Response_message_members = {
  "drone_assembly_cell__srv",  // message namespace
  "MoveToStation_Response",  // message name
  2,  // number of fields
  sizeof(drone_assembly_cell__srv__MoveToStation_Response),
  false,  // has_any_key_member_
  drone_assembly_cell__srv__MoveToStation_Response__rosidl_typesupport_introspection_c__MoveToStation_Response_message_member_array,  // message members
  drone_assembly_cell__srv__MoveToStation_Response__rosidl_typesupport_introspection_c__MoveToStation_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  drone_assembly_cell__srv__MoveToStation_Response__rosidl_typesupport_introspection_c__MoveToStation_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t drone_assembly_cell__srv__MoveToStation_Response__rosidl_typesupport_introspection_c__MoveToStation_Response_message_type_support_handle = {
  0,
  &drone_assembly_cell__srv__MoveToStation_Response__rosidl_typesupport_introspection_c__MoveToStation_Response_message_members,
  get_message_typesupport_handle_function,
  &drone_assembly_cell__srv__MoveToStation_Response__get_type_hash,
  &drone_assembly_cell__srv__MoveToStation_Response__get_type_description,
  &drone_assembly_cell__srv__MoveToStation_Response__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_drone_assembly_cell
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, drone_assembly_cell, srv, MoveToStation_Response)() {
  if (!drone_assembly_cell__srv__MoveToStation_Response__rosidl_typesupport_introspection_c__MoveToStation_Response_message_type_support_handle.typesupport_identifier) {
    drone_assembly_cell__srv__MoveToStation_Response__rosidl_typesupport_introspection_c__MoveToStation_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &drone_assembly_cell__srv__MoveToStation_Response__rosidl_typesupport_introspection_c__MoveToStation_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "drone_assembly_cell/srv/detail/move_to_station__rosidl_typesupport_introspection_c.h"
// already included above
// #include "drone_assembly_cell/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "drone_assembly_cell/srv/detail/move_to_station__functions.h"
// already included above
// #include "drone_assembly_cell/srv/detail/move_to_station__struct.h"


// Include directives for member types
// Member `info`
#include "service_msgs/msg/service_event_info.h"
// Member `info`
#include "service_msgs/msg/detail/service_event_info__rosidl_typesupport_introspection_c.h"
// Member `request`
// Member `response`
#include "drone_assembly_cell/srv/move_to_station.h"
// Member `request`
// Member `response`
// already included above
// #include "drone_assembly_cell/srv/detail/move_to_station__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void drone_assembly_cell__srv__MoveToStation_Event__rosidl_typesupport_introspection_c__MoveToStation_Event_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  drone_assembly_cell__srv__MoveToStation_Event__init(message_memory);
}

void drone_assembly_cell__srv__MoveToStation_Event__rosidl_typesupport_introspection_c__MoveToStation_Event_fini_function(void * message_memory)
{
  drone_assembly_cell__srv__MoveToStation_Event__fini(message_memory);
}

size_t drone_assembly_cell__srv__MoveToStation_Event__rosidl_typesupport_introspection_c__size_function__MoveToStation_Event__request(
  const void * untyped_member)
{
  const drone_assembly_cell__srv__MoveToStation_Request__Sequence * member =
    (const drone_assembly_cell__srv__MoveToStation_Request__Sequence *)(untyped_member);
  return member->size;
}

const void * drone_assembly_cell__srv__MoveToStation_Event__rosidl_typesupport_introspection_c__get_const_function__MoveToStation_Event__request(
  const void * untyped_member, size_t index)
{
  const drone_assembly_cell__srv__MoveToStation_Request__Sequence * member =
    (const drone_assembly_cell__srv__MoveToStation_Request__Sequence *)(untyped_member);
  return &member->data[index];
}

void * drone_assembly_cell__srv__MoveToStation_Event__rosidl_typesupport_introspection_c__get_function__MoveToStation_Event__request(
  void * untyped_member, size_t index)
{
  drone_assembly_cell__srv__MoveToStation_Request__Sequence * member =
    (drone_assembly_cell__srv__MoveToStation_Request__Sequence *)(untyped_member);
  return &member->data[index];
}

void drone_assembly_cell__srv__MoveToStation_Event__rosidl_typesupport_introspection_c__fetch_function__MoveToStation_Event__request(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const drone_assembly_cell__srv__MoveToStation_Request * item =
    ((const drone_assembly_cell__srv__MoveToStation_Request *)
    drone_assembly_cell__srv__MoveToStation_Event__rosidl_typesupport_introspection_c__get_const_function__MoveToStation_Event__request(untyped_member, index));
  drone_assembly_cell__srv__MoveToStation_Request * value =
    (drone_assembly_cell__srv__MoveToStation_Request *)(untyped_value);
  *value = *item;
}

void drone_assembly_cell__srv__MoveToStation_Event__rosidl_typesupport_introspection_c__assign_function__MoveToStation_Event__request(
  void * untyped_member, size_t index, const void * untyped_value)
{
  drone_assembly_cell__srv__MoveToStation_Request * item =
    ((drone_assembly_cell__srv__MoveToStation_Request *)
    drone_assembly_cell__srv__MoveToStation_Event__rosidl_typesupport_introspection_c__get_function__MoveToStation_Event__request(untyped_member, index));
  const drone_assembly_cell__srv__MoveToStation_Request * value =
    (const drone_assembly_cell__srv__MoveToStation_Request *)(untyped_value);
  *item = *value;
}

bool drone_assembly_cell__srv__MoveToStation_Event__rosidl_typesupport_introspection_c__resize_function__MoveToStation_Event__request(
  void * untyped_member, size_t size)
{
  drone_assembly_cell__srv__MoveToStation_Request__Sequence * member =
    (drone_assembly_cell__srv__MoveToStation_Request__Sequence *)(untyped_member);
  drone_assembly_cell__srv__MoveToStation_Request__Sequence__fini(member);
  return drone_assembly_cell__srv__MoveToStation_Request__Sequence__init(member, size);
}

size_t drone_assembly_cell__srv__MoveToStation_Event__rosidl_typesupport_introspection_c__size_function__MoveToStation_Event__response(
  const void * untyped_member)
{
  const drone_assembly_cell__srv__MoveToStation_Response__Sequence * member =
    (const drone_assembly_cell__srv__MoveToStation_Response__Sequence *)(untyped_member);
  return member->size;
}

const void * drone_assembly_cell__srv__MoveToStation_Event__rosidl_typesupport_introspection_c__get_const_function__MoveToStation_Event__response(
  const void * untyped_member, size_t index)
{
  const drone_assembly_cell__srv__MoveToStation_Response__Sequence * member =
    (const drone_assembly_cell__srv__MoveToStation_Response__Sequence *)(untyped_member);
  return &member->data[index];
}

void * drone_assembly_cell__srv__MoveToStation_Event__rosidl_typesupport_introspection_c__get_function__MoveToStation_Event__response(
  void * untyped_member, size_t index)
{
  drone_assembly_cell__srv__MoveToStation_Response__Sequence * member =
    (drone_assembly_cell__srv__MoveToStation_Response__Sequence *)(untyped_member);
  return &member->data[index];
}

void drone_assembly_cell__srv__MoveToStation_Event__rosidl_typesupport_introspection_c__fetch_function__MoveToStation_Event__response(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const drone_assembly_cell__srv__MoveToStation_Response * item =
    ((const drone_assembly_cell__srv__MoveToStation_Response *)
    drone_assembly_cell__srv__MoveToStation_Event__rosidl_typesupport_introspection_c__get_const_function__MoveToStation_Event__response(untyped_member, index));
  drone_assembly_cell__srv__MoveToStation_Response * value =
    (drone_assembly_cell__srv__MoveToStation_Response *)(untyped_value);
  *value = *item;
}

void drone_assembly_cell__srv__MoveToStation_Event__rosidl_typesupport_introspection_c__assign_function__MoveToStation_Event__response(
  void * untyped_member, size_t index, const void * untyped_value)
{
  drone_assembly_cell__srv__MoveToStation_Response * item =
    ((drone_assembly_cell__srv__MoveToStation_Response *)
    drone_assembly_cell__srv__MoveToStation_Event__rosidl_typesupport_introspection_c__get_function__MoveToStation_Event__response(untyped_member, index));
  const drone_assembly_cell__srv__MoveToStation_Response * value =
    (const drone_assembly_cell__srv__MoveToStation_Response *)(untyped_value);
  *item = *value;
}

bool drone_assembly_cell__srv__MoveToStation_Event__rosidl_typesupport_introspection_c__resize_function__MoveToStation_Event__response(
  void * untyped_member, size_t size)
{
  drone_assembly_cell__srv__MoveToStation_Response__Sequence * member =
    (drone_assembly_cell__srv__MoveToStation_Response__Sequence *)(untyped_member);
  drone_assembly_cell__srv__MoveToStation_Response__Sequence__fini(member);
  return drone_assembly_cell__srv__MoveToStation_Response__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember drone_assembly_cell__srv__MoveToStation_Event__rosidl_typesupport_introspection_c__MoveToStation_Event_message_member_array[3] = {
  {
    "info",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(drone_assembly_cell__srv__MoveToStation_Event, info),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "request",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(drone_assembly_cell__srv__MoveToStation_Event, request),  // bytes offset in struct
    NULL,  // default value
    drone_assembly_cell__srv__MoveToStation_Event__rosidl_typesupport_introspection_c__size_function__MoveToStation_Event__request,  // size() function pointer
    drone_assembly_cell__srv__MoveToStation_Event__rosidl_typesupport_introspection_c__get_const_function__MoveToStation_Event__request,  // get_const(index) function pointer
    drone_assembly_cell__srv__MoveToStation_Event__rosidl_typesupport_introspection_c__get_function__MoveToStation_Event__request,  // get(index) function pointer
    drone_assembly_cell__srv__MoveToStation_Event__rosidl_typesupport_introspection_c__fetch_function__MoveToStation_Event__request,  // fetch(index, &value) function pointer
    drone_assembly_cell__srv__MoveToStation_Event__rosidl_typesupport_introspection_c__assign_function__MoveToStation_Event__request,  // assign(index, value) function pointer
    drone_assembly_cell__srv__MoveToStation_Event__rosidl_typesupport_introspection_c__resize_function__MoveToStation_Event__request  // resize(index) function pointer
  },
  {
    "response",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(drone_assembly_cell__srv__MoveToStation_Event, response),  // bytes offset in struct
    NULL,  // default value
    drone_assembly_cell__srv__MoveToStation_Event__rosidl_typesupport_introspection_c__size_function__MoveToStation_Event__response,  // size() function pointer
    drone_assembly_cell__srv__MoveToStation_Event__rosidl_typesupport_introspection_c__get_const_function__MoveToStation_Event__response,  // get_const(index) function pointer
    drone_assembly_cell__srv__MoveToStation_Event__rosidl_typesupport_introspection_c__get_function__MoveToStation_Event__response,  // get(index) function pointer
    drone_assembly_cell__srv__MoveToStation_Event__rosidl_typesupport_introspection_c__fetch_function__MoveToStation_Event__response,  // fetch(index, &value) function pointer
    drone_assembly_cell__srv__MoveToStation_Event__rosidl_typesupport_introspection_c__assign_function__MoveToStation_Event__response,  // assign(index, value) function pointer
    drone_assembly_cell__srv__MoveToStation_Event__rosidl_typesupport_introspection_c__resize_function__MoveToStation_Event__response  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers drone_assembly_cell__srv__MoveToStation_Event__rosidl_typesupport_introspection_c__MoveToStation_Event_message_members = {
  "drone_assembly_cell__srv",  // message namespace
  "MoveToStation_Event",  // message name
  3,  // number of fields
  sizeof(drone_assembly_cell__srv__MoveToStation_Event),
  false,  // has_any_key_member_
  drone_assembly_cell__srv__MoveToStation_Event__rosidl_typesupport_introspection_c__MoveToStation_Event_message_member_array,  // message members
  drone_assembly_cell__srv__MoveToStation_Event__rosidl_typesupport_introspection_c__MoveToStation_Event_init_function,  // function to initialize message memory (memory has to be allocated)
  drone_assembly_cell__srv__MoveToStation_Event__rosidl_typesupport_introspection_c__MoveToStation_Event_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t drone_assembly_cell__srv__MoveToStation_Event__rosidl_typesupport_introspection_c__MoveToStation_Event_message_type_support_handle = {
  0,
  &drone_assembly_cell__srv__MoveToStation_Event__rosidl_typesupport_introspection_c__MoveToStation_Event_message_members,
  get_message_typesupport_handle_function,
  &drone_assembly_cell__srv__MoveToStation_Event__get_type_hash,
  &drone_assembly_cell__srv__MoveToStation_Event__get_type_description,
  &drone_assembly_cell__srv__MoveToStation_Event__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_drone_assembly_cell
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, drone_assembly_cell, srv, MoveToStation_Event)() {
  drone_assembly_cell__srv__MoveToStation_Event__rosidl_typesupport_introspection_c__MoveToStation_Event_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, service_msgs, msg, ServiceEventInfo)();
  drone_assembly_cell__srv__MoveToStation_Event__rosidl_typesupport_introspection_c__MoveToStation_Event_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, drone_assembly_cell, srv, MoveToStation_Request)();
  drone_assembly_cell__srv__MoveToStation_Event__rosidl_typesupport_introspection_c__MoveToStation_Event_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, drone_assembly_cell, srv, MoveToStation_Response)();
  if (!drone_assembly_cell__srv__MoveToStation_Event__rosidl_typesupport_introspection_c__MoveToStation_Event_message_type_support_handle.typesupport_identifier) {
    drone_assembly_cell__srv__MoveToStation_Event__rosidl_typesupport_introspection_c__MoveToStation_Event_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &drone_assembly_cell__srv__MoveToStation_Event__rosidl_typesupport_introspection_c__MoveToStation_Event_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "drone_assembly_cell/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "drone_assembly_cell/srv/detail/move_to_station__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers drone_assembly_cell__srv__detail__move_to_station__rosidl_typesupport_introspection_c__MoveToStation_service_members = {
  "drone_assembly_cell__srv",  // service namespace
  "MoveToStation",  // service name
  // the following fields are initialized below on first access
  NULL,  // request message
  // drone_assembly_cell__srv__detail__move_to_station__rosidl_typesupport_introspection_c__MoveToStation_Request_message_type_support_handle,
  NULL,  // response message
  // drone_assembly_cell__srv__detail__move_to_station__rosidl_typesupport_introspection_c__MoveToStation_Response_message_type_support_handle
  NULL  // event_message
  // drone_assembly_cell__srv__detail__move_to_station__rosidl_typesupport_introspection_c__MoveToStation_Response_message_type_support_handle
};


static rosidl_service_type_support_t drone_assembly_cell__srv__detail__move_to_station__rosidl_typesupport_introspection_c__MoveToStation_service_type_support_handle = {
  0,
  &drone_assembly_cell__srv__detail__move_to_station__rosidl_typesupport_introspection_c__MoveToStation_service_members,
  get_service_typesupport_handle_function,
  &drone_assembly_cell__srv__MoveToStation_Request__rosidl_typesupport_introspection_c__MoveToStation_Request_message_type_support_handle,
  &drone_assembly_cell__srv__MoveToStation_Response__rosidl_typesupport_introspection_c__MoveToStation_Response_message_type_support_handle,
  &drone_assembly_cell__srv__MoveToStation_Event__rosidl_typesupport_introspection_c__MoveToStation_Event_message_type_support_handle,
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_CREATE_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    drone_assembly_cell,
    srv,
    MoveToStation
  ),
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_DESTROY_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    drone_assembly_cell,
    srv,
    MoveToStation
  ),
  &drone_assembly_cell__srv__MoveToStation__get_type_hash,
  &drone_assembly_cell__srv__MoveToStation__get_type_description,
  &drone_assembly_cell__srv__MoveToStation__get_type_description_sources,
};

// Forward declaration of message type support functions for service members
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, drone_assembly_cell, srv, MoveToStation_Request)(void);

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, drone_assembly_cell, srv, MoveToStation_Response)(void);

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, drone_assembly_cell, srv, MoveToStation_Event)(void);

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_drone_assembly_cell
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, drone_assembly_cell, srv, MoveToStation)(void) {
  if (!drone_assembly_cell__srv__detail__move_to_station__rosidl_typesupport_introspection_c__MoveToStation_service_type_support_handle.typesupport_identifier) {
    drone_assembly_cell__srv__detail__move_to_station__rosidl_typesupport_introspection_c__MoveToStation_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)drone_assembly_cell__srv__detail__move_to_station__rosidl_typesupport_introspection_c__MoveToStation_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, drone_assembly_cell, srv, MoveToStation_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, drone_assembly_cell, srv, MoveToStation_Response)()->data;
  }
  if (!service_members->event_members_) {
    service_members->event_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, drone_assembly_cell, srv, MoveToStation_Event)()->data;
  }

  return &drone_assembly_cell__srv__detail__move_to_station__rosidl_typesupport_introspection_c__MoveToStation_service_type_support_handle;
}
