// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from drone_assembly_cell:action/PickPart.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "drone_assembly_cell/action/pick_part.hpp"


#ifndef DRONE_ASSEMBLY_CELL__ACTION__DETAIL__PICK_PART__TRAITS_HPP_
#define DRONE_ASSEMBLY_CELL__ACTION__DETAIL__PICK_PART__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "drone_assembly_cell/action/detail/pick_part__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'source_pose'
#include "geometry_msgs/msg/detail/pose_stamped__traits.hpp"

namespace drone_assembly_cell
{

namespace action
{

inline void to_flow_style_yaml(
  const PickPart_Goal & msg,
  std::ostream & out)
{
  out << "{";
  // member: source_pose
  {
    out << "source_pose: ";
    to_flow_style_yaml(msg.source_pose, out);
    out << ", ";
  }

  // member: approach_height
  {
    out << "approach_height: ";
    rosidl_generator_traits::value_to_yaml(msg.approach_height, out);
    out << ", ";
  }

  // member: gripper_close_pos
  {
    out << "gripper_close_pos: ";
    rosidl_generator_traits::value_to_yaml(msg.gripper_close_pos, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const PickPart_Goal & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: source_pose
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "source_pose:\n";
    to_block_style_yaml(msg.source_pose, out, indentation + 2);
  }

  // member: approach_height
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "approach_height: ";
    rosidl_generator_traits::value_to_yaml(msg.approach_height, out);
    out << "\n";
  }

  // member: gripper_close_pos
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "gripper_close_pos: ";
    rosidl_generator_traits::value_to_yaml(msg.gripper_close_pos, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const PickPart_Goal & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace drone_assembly_cell

namespace rosidl_generator_traits
{

[[deprecated("use drone_assembly_cell::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const drone_assembly_cell::action::PickPart_Goal & msg,
  std::ostream & out, size_t indentation = 0)
{
  drone_assembly_cell::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use drone_assembly_cell::action::to_yaml() instead")]]
inline std::string to_yaml(const drone_assembly_cell::action::PickPart_Goal & msg)
{
  return drone_assembly_cell::action::to_yaml(msg);
}

template<>
inline const char * data_type<drone_assembly_cell::action::PickPart_Goal>()
{
  return "drone_assembly_cell::action::PickPart_Goal";
}

template<>
inline const char * name<drone_assembly_cell::action::PickPart_Goal>()
{
  return "drone_assembly_cell/action/PickPart_Goal";
}

template<>
struct has_fixed_size<drone_assembly_cell::action::PickPart_Goal>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::PoseStamped>::value> {};

template<>
struct has_bounded_size<drone_assembly_cell::action::PickPart_Goal>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::PoseStamped>::value> {};

template<>
struct is_message<drone_assembly_cell::action::PickPart_Goal>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace drone_assembly_cell
{

namespace action
{

inline void to_flow_style_yaml(
  const PickPart_Result & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << ", ";
  }

  // member: message
  {
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const PickPart_Result & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << "\n";
  }

  // member: message
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const PickPart_Result & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace drone_assembly_cell

namespace rosidl_generator_traits
{

[[deprecated("use drone_assembly_cell::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const drone_assembly_cell::action::PickPart_Result & msg,
  std::ostream & out, size_t indentation = 0)
{
  drone_assembly_cell::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use drone_assembly_cell::action::to_yaml() instead")]]
inline std::string to_yaml(const drone_assembly_cell::action::PickPart_Result & msg)
{
  return drone_assembly_cell::action::to_yaml(msg);
}

template<>
inline const char * data_type<drone_assembly_cell::action::PickPart_Result>()
{
  return "drone_assembly_cell::action::PickPart_Result";
}

template<>
inline const char * name<drone_assembly_cell::action::PickPart_Result>()
{
  return "drone_assembly_cell/action/PickPart_Result";
}

template<>
struct has_fixed_size<drone_assembly_cell::action::PickPart_Result>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<drone_assembly_cell::action::PickPart_Result>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<drone_assembly_cell::action::PickPart_Result>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace drone_assembly_cell
{

namespace action
{

inline void to_flow_style_yaml(
  const PickPart_Feedback & msg,
  std::ostream & out)
{
  out << "{";
  // member: phase
  {
    out << "phase: ";
    rosidl_generator_traits::value_to_yaml(msg.phase, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const PickPart_Feedback & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: phase
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "phase: ";
    rosidl_generator_traits::value_to_yaml(msg.phase, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const PickPart_Feedback & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace drone_assembly_cell

namespace rosidl_generator_traits
{

[[deprecated("use drone_assembly_cell::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const drone_assembly_cell::action::PickPart_Feedback & msg,
  std::ostream & out, size_t indentation = 0)
{
  drone_assembly_cell::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use drone_assembly_cell::action::to_yaml() instead")]]
inline std::string to_yaml(const drone_assembly_cell::action::PickPart_Feedback & msg)
{
  return drone_assembly_cell::action::to_yaml(msg);
}

template<>
inline const char * data_type<drone_assembly_cell::action::PickPart_Feedback>()
{
  return "drone_assembly_cell::action::PickPart_Feedback";
}

template<>
inline const char * name<drone_assembly_cell::action::PickPart_Feedback>()
{
  return "drone_assembly_cell/action/PickPart_Feedback";
}

template<>
struct has_fixed_size<drone_assembly_cell::action::PickPart_Feedback>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<drone_assembly_cell::action::PickPart_Feedback>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<drone_assembly_cell::action::PickPart_Feedback>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"
// Member 'goal'
#include "drone_assembly_cell/action/detail/pick_part__traits.hpp"

namespace drone_assembly_cell
{

namespace action
{

inline void to_flow_style_yaml(
  const PickPart_SendGoal_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: goal_id
  {
    out << "goal_id: ";
    to_flow_style_yaml(msg.goal_id, out);
    out << ", ";
  }

  // member: goal
  {
    out << "goal: ";
    to_flow_style_yaml(msg.goal, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const PickPart_SendGoal_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: goal_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_id:\n";
    to_block_style_yaml(msg.goal_id, out, indentation + 2);
  }

  // member: goal
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal:\n";
    to_block_style_yaml(msg.goal, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const PickPart_SendGoal_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace drone_assembly_cell

namespace rosidl_generator_traits
{

[[deprecated("use drone_assembly_cell::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const drone_assembly_cell::action::PickPart_SendGoal_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  drone_assembly_cell::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use drone_assembly_cell::action::to_yaml() instead")]]
inline std::string to_yaml(const drone_assembly_cell::action::PickPart_SendGoal_Request & msg)
{
  return drone_assembly_cell::action::to_yaml(msg);
}

template<>
inline const char * data_type<drone_assembly_cell::action::PickPart_SendGoal_Request>()
{
  return "drone_assembly_cell::action::PickPart_SendGoal_Request";
}

template<>
inline const char * name<drone_assembly_cell::action::PickPart_SendGoal_Request>()
{
  return "drone_assembly_cell/action/PickPart_SendGoal_Request";
}

template<>
struct has_fixed_size<drone_assembly_cell::action::PickPart_SendGoal_Request>
  : std::integral_constant<bool, has_fixed_size<drone_assembly_cell::action::PickPart_Goal>::value && has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<drone_assembly_cell::action::PickPart_SendGoal_Request>
  : std::integral_constant<bool, has_bounded_size<drone_assembly_cell::action::PickPart_Goal>::value && has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<drone_assembly_cell::action::PickPart_SendGoal_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__traits.hpp"

namespace drone_assembly_cell
{

namespace action
{

inline void to_flow_style_yaml(
  const PickPart_SendGoal_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: accepted
  {
    out << "accepted: ";
    rosidl_generator_traits::value_to_yaml(msg.accepted, out);
    out << ", ";
  }

  // member: stamp
  {
    out << "stamp: ";
    to_flow_style_yaml(msg.stamp, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const PickPart_SendGoal_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: accepted
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "accepted: ";
    rosidl_generator_traits::value_to_yaml(msg.accepted, out);
    out << "\n";
  }

  // member: stamp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "stamp:\n";
    to_block_style_yaml(msg.stamp, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const PickPart_SendGoal_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace drone_assembly_cell

namespace rosidl_generator_traits
{

[[deprecated("use drone_assembly_cell::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const drone_assembly_cell::action::PickPart_SendGoal_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  drone_assembly_cell::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use drone_assembly_cell::action::to_yaml() instead")]]
inline std::string to_yaml(const drone_assembly_cell::action::PickPart_SendGoal_Response & msg)
{
  return drone_assembly_cell::action::to_yaml(msg);
}

template<>
inline const char * data_type<drone_assembly_cell::action::PickPart_SendGoal_Response>()
{
  return "drone_assembly_cell::action::PickPart_SendGoal_Response";
}

template<>
inline const char * name<drone_assembly_cell::action::PickPart_SendGoal_Response>()
{
  return "drone_assembly_cell/action/PickPart_SendGoal_Response";
}

template<>
struct has_fixed_size<drone_assembly_cell::action::PickPart_SendGoal_Response>
  : std::integral_constant<bool, has_fixed_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct has_bounded_size<drone_assembly_cell::action::PickPart_SendGoal_Response>
  : std::integral_constant<bool, has_bounded_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct is_message<drone_assembly_cell::action::PickPart_SendGoal_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__traits.hpp"

namespace drone_assembly_cell
{

namespace action
{

inline void to_flow_style_yaml(
  const PickPart_SendGoal_Event & msg,
  std::ostream & out)
{
  out << "{";
  // member: info
  {
    out << "info: ";
    to_flow_style_yaml(msg.info, out);
    out << ", ";
  }

  // member: request
  {
    if (msg.request.size() == 0) {
      out << "request: []";
    } else {
      out << "request: [";
      size_t pending_items = msg.request.size();
      for (auto item : msg.request) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: response
  {
    if (msg.response.size() == 0) {
      out << "response: []";
    } else {
      out << "response: [";
      size_t pending_items = msg.response.size();
      for (auto item : msg.response) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const PickPart_SendGoal_Event & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: info
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "info:\n";
    to_block_style_yaml(msg.info, out, indentation + 2);
  }

  // member: request
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.request.size() == 0) {
      out << "request: []\n";
    } else {
      out << "request:\n";
      for (auto item : msg.request) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: response
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.response.size() == 0) {
      out << "response: []\n";
    } else {
      out << "response:\n";
      for (auto item : msg.response) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const PickPart_SendGoal_Event & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace drone_assembly_cell

namespace rosidl_generator_traits
{

[[deprecated("use drone_assembly_cell::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const drone_assembly_cell::action::PickPart_SendGoal_Event & msg,
  std::ostream & out, size_t indentation = 0)
{
  drone_assembly_cell::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use drone_assembly_cell::action::to_yaml() instead")]]
inline std::string to_yaml(const drone_assembly_cell::action::PickPart_SendGoal_Event & msg)
{
  return drone_assembly_cell::action::to_yaml(msg);
}

template<>
inline const char * data_type<drone_assembly_cell::action::PickPart_SendGoal_Event>()
{
  return "drone_assembly_cell::action::PickPart_SendGoal_Event";
}

template<>
inline const char * name<drone_assembly_cell::action::PickPart_SendGoal_Event>()
{
  return "drone_assembly_cell/action/PickPart_SendGoal_Event";
}

template<>
struct has_fixed_size<drone_assembly_cell::action::PickPart_SendGoal_Event>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<drone_assembly_cell::action::PickPart_SendGoal_Event>
  : std::integral_constant<bool, has_bounded_size<drone_assembly_cell::action::PickPart_SendGoal_Request>::value && has_bounded_size<drone_assembly_cell::action::PickPart_SendGoal_Response>::value && has_bounded_size<service_msgs::msg::ServiceEventInfo>::value> {};

template<>
struct is_message<drone_assembly_cell::action::PickPart_SendGoal_Event>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<drone_assembly_cell::action::PickPart_SendGoal>()
{
  return "drone_assembly_cell::action::PickPart_SendGoal";
}

template<>
inline const char * name<drone_assembly_cell::action::PickPart_SendGoal>()
{
  return "drone_assembly_cell/action/PickPart_SendGoal";
}

template<>
struct has_fixed_size<drone_assembly_cell::action::PickPart_SendGoal>
  : std::integral_constant<
    bool,
    has_fixed_size<drone_assembly_cell::action::PickPart_SendGoal_Request>::value &&
    has_fixed_size<drone_assembly_cell::action::PickPart_SendGoal_Response>::value
  >
{
};

template<>
struct has_bounded_size<drone_assembly_cell::action::PickPart_SendGoal>
  : std::integral_constant<
    bool,
    has_bounded_size<drone_assembly_cell::action::PickPart_SendGoal_Request>::value &&
    has_bounded_size<drone_assembly_cell::action::PickPart_SendGoal_Response>::value
  >
{
};

template<>
struct is_service<drone_assembly_cell::action::PickPart_SendGoal>
  : std::true_type
{
};

template<>
struct is_service_request<drone_assembly_cell::action::PickPart_SendGoal_Request>
  : std::true_type
{
};

template<>
struct is_service_response<drone_assembly_cell::action::PickPart_SendGoal_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"

namespace drone_assembly_cell
{

namespace action
{

inline void to_flow_style_yaml(
  const PickPart_GetResult_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: goal_id
  {
    out << "goal_id: ";
    to_flow_style_yaml(msg.goal_id, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const PickPart_GetResult_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: goal_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_id:\n";
    to_block_style_yaml(msg.goal_id, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const PickPart_GetResult_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace drone_assembly_cell

namespace rosidl_generator_traits
{

[[deprecated("use drone_assembly_cell::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const drone_assembly_cell::action::PickPart_GetResult_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  drone_assembly_cell::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use drone_assembly_cell::action::to_yaml() instead")]]
inline std::string to_yaml(const drone_assembly_cell::action::PickPart_GetResult_Request & msg)
{
  return drone_assembly_cell::action::to_yaml(msg);
}

template<>
inline const char * data_type<drone_assembly_cell::action::PickPart_GetResult_Request>()
{
  return "drone_assembly_cell::action::PickPart_GetResult_Request";
}

template<>
inline const char * name<drone_assembly_cell::action::PickPart_GetResult_Request>()
{
  return "drone_assembly_cell/action/PickPart_GetResult_Request";
}

template<>
struct has_fixed_size<drone_assembly_cell::action::PickPart_GetResult_Request>
  : std::integral_constant<bool, has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<drone_assembly_cell::action::PickPart_GetResult_Request>
  : std::integral_constant<bool, has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<drone_assembly_cell::action::PickPart_GetResult_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'result'
// already included above
// #include "drone_assembly_cell/action/detail/pick_part__traits.hpp"

namespace drone_assembly_cell
{

namespace action
{

inline void to_flow_style_yaml(
  const PickPart_GetResult_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: status
  {
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << ", ";
  }

  // member: result
  {
    out << "result: ";
    to_flow_style_yaml(msg.result, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const PickPart_GetResult_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << "\n";
  }

  // member: result
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "result:\n";
    to_block_style_yaml(msg.result, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const PickPart_GetResult_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace drone_assembly_cell

namespace rosidl_generator_traits
{

[[deprecated("use drone_assembly_cell::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const drone_assembly_cell::action::PickPart_GetResult_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  drone_assembly_cell::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use drone_assembly_cell::action::to_yaml() instead")]]
inline std::string to_yaml(const drone_assembly_cell::action::PickPart_GetResult_Response & msg)
{
  return drone_assembly_cell::action::to_yaml(msg);
}

template<>
inline const char * data_type<drone_assembly_cell::action::PickPart_GetResult_Response>()
{
  return "drone_assembly_cell::action::PickPart_GetResult_Response";
}

template<>
inline const char * name<drone_assembly_cell::action::PickPart_GetResult_Response>()
{
  return "drone_assembly_cell/action/PickPart_GetResult_Response";
}

template<>
struct has_fixed_size<drone_assembly_cell::action::PickPart_GetResult_Response>
  : std::integral_constant<bool, has_fixed_size<drone_assembly_cell::action::PickPart_Result>::value> {};

template<>
struct has_bounded_size<drone_assembly_cell::action::PickPart_GetResult_Response>
  : std::integral_constant<bool, has_bounded_size<drone_assembly_cell::action::PickPart_Result>::value> {};

template<>
struct is_message<drone_assembly_cell::action::PickPart_GetResult_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'info'
// already included above
// #include "service_msgs/msg/detail/service_event_info__traits.hpp"

namespace drone_assembly_cell
{

namespace action
{

inline void to_flow_style_yaml(
  const PickPart_GetResult_Event & msg,
  std::ostream & out)
{
  out << "{";
  // member: info
  {
    out << "info: ";
    to_flow_style_yaml(msg.info, out);
    out << ", ";
  }

  // member: request
  {
    if (msg.request.size() == 0) {
      out << "request: []";
    } else {
      out << "request: [";
      size_t pending_items = msg.request.size();
      for (auto item : msg.request) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: response
  {
    if (msg.response.size() == 0) {
      out << "response: []";
    } else {
      out << "response: [";
      size_t pending_items = msg.response.size();
      for (auto item : msg.response) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const PickPart_GetResult_Event & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: info
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "info:\n";
    to_block_style_yaml(msg.info, out, indentation + 2);
  }

  // member: request
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.request.size() == 0) {
      out << "request: []\n";
    } else {
      out << "request:\n";
      for (auto item : msg.request) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: response
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.response.size() == 0) {
      out << "response: []\n";
    } else {
      out << "response:\n";
      for (auto item : msg.response) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const PickPart_GetResult_Event & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace drone_assembly_cell

namespace rosidl_generator_traits
{

[[deprecated("use drone_assembly_cell::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const drone_assembly_cell::action::PickPart_GetResult_Event & msg,
  std::ostream & out, size_t indentation = 0)
{
  drone_assembly_cell::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use drone_assembly_cell::action::to_yaml() instead")]]
inline std::string to_yaml(const drone_assembly_cell::action::PickPart_GetResult_Event & msg)
{
  return drone_assembly_cell::action::to_yaml(msg);
}

template<>
inline const char * data_type<drone_assembly_cell::action::PickPart_GetResult_Event>()
{
  return "drone_assembly_cell::action::PickPart_GetResult_Event";
}

template<>
inline const char * name<drone_assembly_cell::action::PickPart_GetResult_Event>()
{
  return "drone_assembly_cell/action/PickPart_GetResult_Event";
}

template<>
struct has_fixed_size<drone_assembly_cell::action::PickPart_GetResult_Event>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<drone_assembly_cell::action::PickPart_GetResult_Event>
  : std::integral_constant<bool, has_bounded_size<drone_assembly_cell::action::PickPart_GetResult_Request>::value && has_bounded_size<drone_assembly_cell::action::PickPart_GetResult_Response>::value && has_bounded_size<service_msgs::msg::ServiceEventInfo>::value> {};

template<>
struct is_message<drone_assembly_cell::action::PickPart_GetResult_Event>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<drone_assembly_cell::action::PickPart_GetResult>()
{
  return "drone_assembly_cell::action::PickPart_GetResult";
}

template<>
inline const char * name<drone_assembly_cell::action::PickPart_GetResult>()
{
  return "drone_assembly_cell/action/PickPart_GetResult";
}

template<>
struct has_fixed_size<drone_assembly_cell::action::PickPart_GetResult>
  : std::integral_constant<
    bool,
    has_fixed_size<drone_assembly_cell::action::PickPart_GetResult_Request>::value &&
    has_fixed_size<drone_assembly_cell::action::PickPart_GetResult_Response>::value
  >
{
};

template<>
struct has_bounded_size<drone_assembly_cell::action::PickPart_GetResult>
  : std::integral_constant<
    bool,
    has_bounded_size<drone_assembly_cell::action::PickPart_GetResult_Request>::value &&
    has_bounded_size<drone_assembly_cell::action::PickPart_GetResult_Response>::value
  >
{
};

template<>
struct is_service<drone_assembly_cell::action::PickPart_GetResult>
  : std::true_type
{
};

template<>
struct is_service_request<drone_assembly_cell::action::PickPart_GetResult_Request>
  : std::true_type
{
};

template<>
struct is_service_response<drone_assembly_cell::action::PickPart_GetResult_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"
// Member 'feedback'
// already included above
// #include "drone_assembly_cell/action/detail/pick_part__traits.hpp"

namespace drone_assembly_cell
{

namespace action
{

inline void to_flow_style_yaml(
  const PickPart_FeedbackMessage & msg,
  std::ostream & out)
{
  out << "{";
  // member: goal_id
  {
    out << "goal_id: ";
    to_flow_style_yaml(msg.goal_id, out);
    out << ", ";
  }

  // member: feedback
  {
    out << "feedback: ";
    to_flow_style_yaml(msg.feedback, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const PickPart_FeedbackMessage & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: goal_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_id:\n";
    to_block_style_yaml(msg.goal_id, out, indentation + 2);
  }

  // member: feedback
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "feedback:\n";
    to_block_style_yaml(msg.feedback, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const PickPart_FeedbackMessage & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace drone_assembly_cell

namespace rosidl_generator_traits
{

[[deprecated("use drone_assembly_cell::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const drone_assembly_cell::action::PickPart_FeedbackMessage & msg,
  std::ostream & out, size_t indentation = 0)
{
  drone_assembly_cell::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use drone_assembly_cell::action::to_yaml() instead")]]
inline std::string to_yaml(const drone_assembly_cell::action::PickPart_FeedbackMessage & msg)
{
  return drone_assembly_cell::action::to_yaml(msg);
}

template<>
inline const char * data_type<drone_assembly_cell::action::PickPart_FeedbackMessage>()
{
  return "drone_assembly_cell::action::PickPart_FeedbackMessage";
}

template<>
inline const char * name<drone_assembly_cell::action::PickPart_FeedbackMessage>()
{
  return "drone_assembly_cell/action/PickPart_FeedbackMessage";
}

template<>
struct has_fixed_size<drone_assembly_cell::action::PickPart_FeedbackMessage>
  : std::integral_constant<bool, has_fixed_size<drone_assembly_cell::action::PickPart_Feedback>::value && has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<drone_assembly_cell::action::PickPart_FeedbackMessage>
  : std::integral_constant<bool, has_bounded_size<drone_assembly_cell::action::PickPart_Feedback>::value && has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<drone_assembly_cell::action::PickPart_FeedbackMessage>
  : std::true_type {};

}  // namespace rosidl_generator_traits


namespace rosidl_generator_traits
{

template<>
struct is_action<drone_assembly_cell::action::PickPart>
  : std::true_type
{
};

template<>
struct is_action_goal<drone_assembly_cell::action::PickPart_Goal>
  : std::true_type
{
};

template<>
struct is_action_result<drone_assembly_cell::action::PickPart_Result>
  : std::true_type
{
};

template<>
struct is_action_feedback<drone_assembly_cell::action::PickPart_Feedback>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits


#endif  // DRONE_ASSEMBLY_CELL__ACTION__DETAIL__PICK_PART__TRAITS_HPP_
