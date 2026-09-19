// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from drone_assembly_cell:srv/MoveToStation.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "drone_assembly_cell/srv/move_to_station.h"


#ifndef DRONE_ASSEMBLY_CELL__SRV__DETAIL__MOVE_TO_STATION__STRUCT_H_
#define DRONE_ASSEMBLY_CELL__SRV__DETAIL__MOVE_TO_STATION__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'drone_id'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/MoveToStation in the package drone_assembly_cell.
typedef struct drone_assembly_cell__srv__MoveToStation_Request
{
  rosidl_runtime_c__String drone_id;
  int32_t target_station;
} drone_assembly_cell__srv__MoveToStation_Request;

// Struct for a sequence of drone_assembly_cell__srv__MoveToStation_Request.
typedef struct drone_assembly_cell__srv__MoveToStation_Request__Sequence
{
  drone_assembly_cell__srv__MoveToStation_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} drone_assembly_cell__srv__MoveToStation_Request__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'message'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in srv/MoveToStation in the package drone_assembly_cell.
typedef struct drone_assembly_cell__srv__MoveToStation_Response
{
  bool success;
  rosidl_runtime_c__String message;
} drone_assembly_cell__srv__MoveToStation_Response;

// Struct for a sequence of drone_assembly_cell__srv__MoveToStation_Response.
typedef struct drone_assembly_cell__srv__MoveToStation_Response__Sequence
{
  drone_assembly_cell__srv__MoveToStation_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} drone_assembly_cell__srv__MoveToStation_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  drone_assembly_cell__srv__MoveToStation_Event__request__MAX_SIZE = 1
};
// response
enum
{
  drone_assembly_cell__srv__MoveToStation_Event__response__MAX_SIZE = 1
};

/// Struct defined in srv/MoveToStation in the package drone_assembly_cell.
typedef struct drone_assembly_cell__srv__MoveToStation_Event
{
  service_msgs__msg__ServiceEventInfo info;
  drone_assembly_cell__srv__MoveToStation_Request__Sequence request;
  drone_assembly_cell__srv__MoveToStation_Response__Sequence response;
} drone_assembly_cell__srv__MoveToStation_Event;

// Struct for a sequence of drone_assembly_cell__srv__MoveToStation_Event.
typedef struct drone_assembly_cell__srv__MoveToStation_Event__Sequence
{
  drone_assembly_cell__srv__MoveToStation_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} drone_assembly_cell__srv__MoveToStation_Event__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // DRONE_ASSEMBLY_CELL__SRV__DETAIL__MOVE_TO_STATION__STRUCT_H_
