// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from drone_assembly_cell:action/PickPart.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "drone_assembly_cell/action/pick_part.h"


#ifndef DRONE_ASSEMBLY_CELL__ACTION__DETAIL__PICK_PART__STRUCT_H_
#define DRONE_ASSEMBLY_CELL__ACTION__DETAIL__PICK_PART__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'source_pose'
#include "geometry_msgs/msg/detail/pose_stamped__struct.h"

/// Struct defined in action/PickPart in the package drone_assembly_cell.
typedef struct drone_assembly_cell__action__PickPart_Goal
{
  geometry_msgs__msg__PoseStamped source_pose;
  double approach_height;
  double gripper_close_pos;
} drone_assembly_cell__action__PickPart_Goal;

// Struct for a sequence of drone_assembly_cell__action__PickPart_Goal.
typedef struct drone_assembly_cell__action__PickPart_Goal__Sequence
{
  drone_assembly_cell__action__PickPart_Goal * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} drone_assembly_cell__action__PickPart_Goal__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'message'
#include "rosidl_runtime_c/string.h"

/// Struct defined in action/PickPart in the package drone_assembly_cell.
typedef struct drone_assembly_cell__action__PickPart_Result
{
  bool success;
  rosidl_runtime_c__String message;
} drone_assembly_cell__action__PickPart_Result;

// Struct for a sequence of drone_assembly_cell__action__PickPart_Result.
typedef struct drone_assembly_cell__action__PickPart_Result__Sequence
{
  drone_assembly_cell__action__PickPart_Result * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} drone_assembly_cell__action__PickPart_Result__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'phase'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in action/PickPart in the package drone_assembly_cell.
typedef struct drone_assembly_cell__action__PickPart_Feedback
{
  rosidl_runtime_c__String phase;
} drone_assembly_cell__action__PickPart_Feedback;

// Struct for a sequence of drone_assembly_cell__action__PickPart_Feedback.
typedef struct drone_assembly_cell__action__PickPart_Feedback__Sequence
{
  drone_assembly_cell__action__PickPart_Feedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} drone_assembly_cell__action__PickPart_Feedback__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'goal'
#include "drone_assembly_cell/action/detail/pick_part__struct.h"

/// Struct defined in action/PickPart in the package drone_assembly_cell.
typedef struct drone_assembly_cell__action__PickPart_SendGoal_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
  drone_assembly_cell__action__PickPart_Goal goal;
} drone_assembly_cell__action__PickPart_SendGoal_Request;

// Struct for a sequence of drone_assembly_cell__action__PickPart_SendGoal_Request.
typedef struct drone_assembly_cell__action__PickPart_SendGoal_Request__Sequence
{
  drone_assembly_cell__action__PickPart_SendGoal_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} drone_assembly_cell__action__PickPart_SendGoal_Request__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in action/PickPart in the package drone_assembly_cell.
typedef struct drone_assembly_cell__action__PickPart_SendGoal_Response
{
  bool accepted;
  builtin_interfaces__msg__Time stamp;
} drone_assembly_cell__action__PickPart_SendGoal_Response;

// Struct for a sequence of drone_assembly_cell__action__PickPart_SendGoal_Response.
typedef struct drone_assembly_cell__action__PickPart_SendGoal_Response__Sequence
{
  drone_assembly_cell__action__PickPart_SendGoal_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} drone_assembly_cell__action__PickPart_SendGoal_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  drone_assembly_cell__action__PickPart_SendGoal_Event__request__MAX_SIZE = 1
};
// response
enum
{
  drone_assembly_cell__action__PickPart_SendGoal_Event__response__MAX_SIZE = 1
};

/// Struct defined in action/PickPart in the package drone_assembly_cell.
typedef struct drone_assembly_cell__action__PickPart_SendGoal_Event
{
  service_msgs__msg__ServiceEventInfo info;
  drone_assembly_cell__action__PickPart_SendGoal_Request__Sequence request;
  drone_assembly_cell__action__PickPart_SendGoal_Response__Sequence response;
} drone_assembly_cell__action__PickPart_SendGoal_Event;

// Struct for a sequence of drone_assembly_cell__action__PickPart_SendGoal_Event.
typedef struct drone_assembly_cell__action__PickPart_SendGoal_Event__Sequence
{
  drone_assembly_cell__action__PickPart_SendGoal_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} drone_assembly_cell__action__PickPart_SendGoal_Event__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"

/// Struct defined in action/PickPart in the package drone_assembly_cell.
typedef struct drone_assembly_cell__action__PickPart_GetResult_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
} drone_assembly_cell__action__PickPart_GetResult_Request;

// Struct for a sequence of drone_assembly_cell__action__PickPart_GetResult_Request.
typedef struct drone_assembly_cell__action__PickPart_GetResult_Request__Sequence
{
  drone_assembly_cell__action__PickPart_GetResult_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} drone_assembly_cell__action__PickPart_GetResult_Request__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'result'
// already included above
// #include "drone_assembly_cell/action/detail/pick_part__struct.h"

/// Struct defined in action/PickPart in the package drone_assembly_cell.
typedef struct drone_assembly_cell__action__PickPart_GetResult_Response
{
  int8_t status;
  drone_assembly_cell__action__PickPart_Result result;
} drone_assembly_cell__action__PickPart_GetResult_Response;

// Struct for a sequence of drone_assembly_cell__action__PickPart_GetResult_Response.
typedef struct drone_assembly_cell__action__PickPart_GetResult_Response__Sequence
{
  drone_assembly_cell__action__PickPart_GetResult_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} drone_assembly_cell__action__PickPart_GetResult_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
// already included above
// #include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  drone_assembly_cell__action__PickPart_GetResult_Event__request__MAX_SIZE = 1
};
// response
enum
{
  drone_assembly_cell__action__PickPart_GetResult_Event__response__MAX_SIZE = 1
};

/// Struct defined in action/PickPart in the package drone_assembly_cell.
typedef struct drone_assembly_cell__action__PickPart_GetResult_Event
{
  service_msgs__msg__ServiceEventInfo info;
  drone_assembly_cell__action__PickPart_GetResult_Request__Sequence request;
  drone_assembly_cell__action__PickPart_GetResult_Response__Sequence response;
} drone_assembly_cell__action__PickPart_GetResult_Event;

// Struct for a sequence of drone_assembly_cell__action__PickPart_GetResult_Event.
typedef struct drone_assembly_cell__action__PickPart_GetResult_Event__Sequence
{
  drone_assembly_cell__action__PickPart_GetResult_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} drone_assembly_cell__action__PickPart_GetResult_Event__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'feedback'
// already included above
// #include "drone_assembly_cell/action/detail/pick_part__struct.h"

/// Struct defined in action/PickPart in the package drone_assembly_cell.
typedef struct drone_assembly_cell__action__PickPart_FeedbackMessage
{
  unique_identifier_msgs__msg__UUID goal_id;
  drone_assembly_cell__action__PickPart_Feedback feedback;
} drone_assembly_cell__action__PickPart_FeedbackMessage;

// Struct for a sequence of drone_assembly_cell__action__PickPart_FeedbackMessage.
typedef struct drone_assembly_cell__action__PickPart_FeedbackMessage__Sequence
{
  drone_assembly_cell__action__PickPart_FeedbackMessage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} drone_assembly_cell__action__PickPart_FeedbackMessage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // DRONE_ASSEMBLY_CELL__ACTION__DETAIL__PICK_PART__STRUCT_H_
