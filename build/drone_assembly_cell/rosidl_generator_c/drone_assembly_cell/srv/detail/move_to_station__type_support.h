// generated from rosidl_generator_c/resource/idl__type_support.h.em
// with input from drone_assembly_cell:srv/MoveToStation.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "drone_assembly_cell/srv/move_to_station.h"


#ifndef DRONE_ASSEMBLY_CELL__SRV__DETAIL__MOVE_TO_STATION__TYPE_SUPPORT_H_
#define DRONE_ASSEMBLY_CELL__SRV__DETAIL__MOVE_TO_STATION__TYPE_SUPPORT_H_

#include "rosidl_typesupport_interface/macros.h"

#include "drone_assembly_cell/msg/rosidl_generator_c__visibility_control.h"

#ifdef __cplusplus
extern "C"
{
#endif

#include "rosidl_runtime_c/message_type_support_struct.h"

// Forward declare the get type support functions for this type.
ROSIDL_GENERATOR_C_PUBLIC_drone_assembly_cell
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
  rosidl_typesupport_c,
  drone_assembly_cell,
  srv,
  MoveToStation_Request
)(void);

// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"

// Forward declare the get type support functions for this type.
ROSIDL_GENERATOR_C_PUBLIC_drone_assembly_cell
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
  rosidl_typesupport_c,
  drone_assembly_cell,
  srv,
  MoveToStation_Response
)(void);

// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"

// Forward declare the get type support functions for this type.
ROSIDL_GENERATOR_C_PUBLIC_drone_assembly_cell
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
  rosidl_typesupport_c,
  drone_assembly_cell,
  srv,
  MoveToStation_Event
)(void);

#include "rosidl_runtime_c/service_type_support_struct.h"

// Forward declare the get type support functions for this type.
ROSIDL_GENERATOR_C_PUBLIC_drone_assembly_cell
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(
  rosidl_typesupport_c,
  drone_assembly_cell,
  srv,
  MoveToStation
)(void);

// Forward declare the function to create a service event message for this type.
ROSIDL_GENERATOR_C_PUBLIC_drone_assembly_cell
void *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_CREATE_EVENT_MESSAGE_SYMBOL_NAME(
  rosidl_typesupport_c,
  drone_assembly_cell,
  srv,
  MoveToStation
)(
  const rosidl_service_introspection_info_t * info,
  rcutils_allocator_t * allocator,
  const void * request_message,
  const void * response_message);

// Forward declare the function to destroy a service event message for this type.
ROSIDL_GENERATOR_C_PUBLIC_drone_assembly_cell
bool
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_DESTROY_EVENT_MESSAGE_SYMBOL_NAME(
  rosidl_typesupport_c,
  drone_assembly_cell,
  srv,
  MoveToStation
)(
  void * event_msg,
  rcutils_allocator_t * allocator);

#ifdef __cplusplus
}
#endif

#endif  // DRONE_ASSEMBLY_CELL__SRV__DETAIL__MOVE_TO_STATION__TYPE_SUPPORT_H_
