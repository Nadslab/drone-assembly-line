// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from drone_assembly_cell:action/PlacePart.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "drone_assembly_cell/action/place_part.hpp"


#ifndef DRONE_ASSEMBLY_CELL__ACTION__DETAIL__PLACE_PART__BUILDER_HPP_
#define DRONE_ASSEMBLY_CELL__ACTION__DETAIL__PLACE_PART__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "drone_assembly_cell/action/detail/place_part__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace drone_assembly_cell
{

namespace action
{

namespace builder
{

class Init_PlacePart_Goal_gripper_open_pos
{
public:
  explicit Init_PlacePart_Goal_gripper_open_pos(::drone_assembly_cell::action::PlacePart_Goal & msg)
  : msg_(msg)
  {}
  ::drone_assembly_cell::action::PlacePart_Goal gripper_open_pos(::drone_assembly_cell::action::PlacePart_Goal::_gripper_open_pos_type arg)
  {
    msg_.gripper_open_pos = std::move(arg);
    return std::move(msg_);
  }

private:
  ::drone_assembly_cell::action::PlacePart_Goal msg_;
};

class Init_PlacePart_Goal_approach_height
{
public:
  explicit Init_PlacePart_Goal_approach_height(::drone_assembly_cell::action::PlacePart_Goal & msg)
  : msg_(msg)
  {}
  Init_PlacePart_Goal_gripper_open_pos approach_height(::drone_assembly_cell::action::PlacePart_Goal::_approach_height_type arg)
  {
    msg_.approach_height = std::move(arg);
    return Init_PlacePart_Goal_gripper_open_pos(msg_);
  }

private:
  ::drone_assembly_cell::action::PlacePart_Goal msg_;
};

class Init_PlacePart_Goal_target_pose
{
public:
  Init_PlacePart_Goal_target_pose()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PlacePart_Goal_approach_height target_pose(::drone_assembly_cell::action::PlacePart_Goal::_target_pose_type arg)
  {
    msg_.target_pose = std::move(arg);
    return Init_PlacePart_Goal_approach_height(msg_);
  }

private:
  ::drone_assembly_cell::action::PlacePart_Goal msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::drone_assembly_cell::action::PlacePart_Goal>()
{
  return drone_assembly_cell::action::builder::Init_PlacePart_Goal_target_pose();
}

}  // namespace drone_assembly_cell


namespace drone_assembly_cell
{

namespace action
{

namespace builder
{

class Init_PlacePart_Result_message
{
public:
  explicit Init_PlacePart_Result_message(::drone_assembly_cell::action::PlacePart_Result & msg)
  : msg_(msg)
  {}
  ::drone_assembly_cell::action::PlacePart_Result message(::drone_assembly_cell::action::PlacePart_Result::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::drone_assembly_cell::action::PlacePart_Result msg_;
};

class Init_PlacePart_Result_success
{
public:
  Init_PlacePart_Result_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PlacePart_Result_message success(::drone_assembly_cell::action::PlacePart_Result::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_PlacePart_Result_message(msg_);
  }

private:
  ::drone_assembly_cell::action::PlacePart_Result msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::drone_assembly_cell::action::PlacePart_Result>()
{
  return drone_assembly_cell::action::builder::Init_PlacePart_Result_success();
}

}  // namespace drone_assembly_cell


namespace drone_assembly_cell
{

namespace action
{

namespace builder
{

class Init_PlacePart_Feedback_phase
{
public:
  Init_PlacePart_Feedback_phase()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::drone_assembly_cell::action::PlacePart_Feedback phase(::drone_assembly_cell::action::PlacePart_Feedback::_phase_type arg)
  {
    msg_.phase = std::move(arg);
    return std::move(msg_);
  }

private:
  ::drone_assembly_cell::action::PlacePart_Feedback msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::drone_assembly_cell::action::PlacePart_Feedback>()
{
  return drone_assembly_cell::action::builder::Init_PlacePart_Feedback_phase();
}

}  // namespace drone_assembly_cell


namespace drone_assembly_cell
{

namespace action
{

namespace builder
{

class Init_PlacePart_SendGoal_Request_goal
{
public:
  explicit Init_PlacePart_SendGoal_Request_goal(::drone_assembly_cell::action::PlacePart_SendGoal_Request & msg)
  : msg_(msg)
  {}
  ::drone_assembly_cell::action::PlacePart_SendGoal_Request goal(::drone_assembly_cell::action::PlacePart_SendGoal_Request::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::drone_assembly_cell::action::PlacePart_SendGoal_Request msg_;
};

class Init_PlacePart_SendGoal_Request_goal_id
{
public:
  Init_PlacePart_SendGoal_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PlacePart_SendGoal_Request_goal goal_id(::drone_assembly_cell::action::PlacePart_SendGoal_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_PlacePart_SendGoal_Request_goal(msg_);
  }

private:
  ::drone_assembly_cell::action::PlacePart_SendGoal_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::drone_assembly_cell::action::PlacePart_SendGoal_Request>()
{
  return drone_assembly_cell::action::builder::Init_PlacePart_SendGoal_Request_goal_id();
}

}  // namespace drone_assembly_cell


namespace drone_assembly_cell
{

namespace action
{

namespace builder
{

class Init_PlacePart_SendGoal_Response_stamp
{
public:
  explicit Init_PlacePart_SendGoal_Response_stamp(::drone_assembly_cell::action::PlacePart_SendGoal_Response & msg)
  : msg_(msg)
  {}
  ::drone_assembly_cell::action::PlacePart_SendGoal_Response stamp(::drone_assembly_cell::action::PlacePart_SendGoal_Response::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::drone_assembly_cell::action::PlacePart_SendGoal_Response msg_;
};

class Init_PlacePart_SendGoal_Response_accepted
{
public:
  Init_PlacePart_SendGoal_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PlacePart_SendGoal_Response_stamp accepted(::drone_assembly_cell::action::PlacePart_SendGoal_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_PlacePart_SendGoal_Response_stamp(msg_);
  }

private:
  ::drone_assembly_cell::action::PlacePart_SendGoal_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::drone_assembly_cell::action::PlacePart_SendGoal_Response>()
{
  return drone_assembly_cell::action::builder::Init_PlacePart_SendGoal_Response_accepted();
}

}  // namespace drone_assembly_cell


namespace drone_assembly_cell
{

namespace action
{

namespace builder
{

class Init_PlacePart_SendGoal_Event_response
{
public:
  explicit Init_PlacePart_SendGoal_Event_response(::drone_assembly_cell::action::PlacePart_SendGoal_Event & msg)
  : msg_(msg)
  {}
  ::drone_assembly_cell::action::PlacePart_SendGoal_Event response(::drone_assembly_cell::action::PlacePart_SendGoal_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::drone_assembly_cell::action::PlacePart_SendGoal_Event msg_;
};

class Init_PlacePart_SendGoal_Event_request
{
public:
  explicit Init_PlacePart_SendGoal_Event_request(::drone_assembly_cell::action::PlacePart_SendGoal_Event & msg)
  : msg_(msg)
  {}
  Init_PlacePart_SendGoal_Event_response request(::drone_assembly_cell::action::PlacePart_SendGoal_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_PlacePart_SendGoal_Event_response(msg_);
  }

private:
  ::drone_assembly_cell::action::PlacePart_SendGoal_Event msg_;
};

class Init_PlacePart_SendGoal_Event_info
{
public:
  Init_PlacePart_SendGoal_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PlacePart_SendGoal_Event_request info(::drone_assembly_cell::action::PlacePart_SendGoal_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_PlacePart_SendGoal_Event_request(msg_);
  }

private:
  ::drone_assembly_cell::action::PlacePart_SendGoal_Event msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::drone_assembly_cell::action::PlacePart_SendGoal_Event>()
{
  return drone_assembly_cell::action::builder::Init_PlacePart_SendGoal_Event_info();
}

}  // namespace drone_assembly_cell


namespace drone_assembly_cell
{

namespace action
{

namespace builder
{

class Init_PlacePart_GetResult_Request_goal_id
{
public:
  Init_PlacePart_GetResult_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::drone_assembly_cell::action::PlacePart_GetResult_Request goal_id(::drone_assembly_cell::action::PlacePart_GetResult_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::drone_assembly_cell::action::PlacePart_GetResult_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::drone_assembly_cell::action::PlacePart_GetResult_Request>()
{
  return drone_assembly_cell::action::builder::Init_PlacePart_GetResult_Request_goal_id();
}

}  // namespace drone_assembly_cell


namespace drone_assembly_cell
{

namespace action
{

namespace builder
{

class Init_PlacePart_GetResult_Response_result
{
public:
  explicit Init_PlacePart_GetResult_Response_result(::drone_assembly_cell::action::PlacePart_GetResult_Response & msg)
  : msg_(msg)
  {}
  ::drone_assembly_cell::action::PlacePart_GetResult_Response result(::drone_assembly_cell::action::PlacePart_GetResult_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::drone_assembly_cell::action::PlacePart_GetResult_Response msg_;
};

class Init_PlacePart_GetResult_Response_status
{
public:
  Init_PlacePart_GetResult_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PlacePart_GetResult_Response_result status(::drone_assembly_cell::action::PlacePart_GetResult_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_PlacePart_GetResult_Response_result(msg_);
  }

private:
  ::drone_assembly_cell::action::PlacePart_GetResult_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::drone_assembly_cell::action::PlacePart_GetResult_Response>()
{
  return drone_assembly_cell::action::builder::Init_PlacePart_GetResult_Response_status();
}

}  // namespace drone_assembly_cell


namespace drone_assembly_cell
{

namespace action
{

namespace builder
{

class Init_PlacePart_GetResult_Event_response
{
public:
  explicit Init_PlacePart_GetResult_Event_response(::drone_assembly_cell::action::PlacePart_GetResult_Event & msg)
  : msg_(msg)
  {}
  ::drone_assembly_cell::action::PlacePart_GetResult_Event response(::drone_assembly_cell::action::PlacePart_GetResult_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::drone_assembly_cell::action::PlacePart_GetResult_Event msg_;
};

class Init_PlacePart_GetResult_Event_request
{
public:
  explicit Init_PlacePart_GetResult_Event_request(::drone_assembly_cell::action::PlacePart_GetResult_Event & msg)
  : msg_(msg)
  {}
  Init_PlacePart_GetResult_Event_response request(::drone_assembly_cell::action::PlacePart_GetResult_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_PlacePart_GetResult_Event_response(msg_);
  }

private:
  ::drone_assembly_cell::action::PlacePart_GetResult_Event msg_;
};

class Init_PlacePart_GetResult_Event_info
{
public:
  Init_PlacePart_GetResult_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PlacePart_GetResult_Event_request info(::drone_assembly_cell::action::PlacePart_GetResult_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_PlacePart_GetResult_Event_request(msg_);
  }

private:
  ::drone_assembly_cell::action::PlacePart_GetResult_Event msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::drone_assembly_cell::action::PlacePart_GetResult_Event>()
{
  return drone_assembly_cell::action::builder::Init_PlacePart_GetResult_Event_info();
}

}  // namespace drone_assembly_cell


namespace drone_assembly_cell
{

namespace action
{

namespace builder
{

class Init_PlacePart_FeedbackMessage_feedback
{
public:
  explicit Init_PlacePart_FeedbackMessage_feedback(::drone_assembly_cell::action::PlacePart_FeedbackMessage & msg)
  : msg_(msg)
  {}
  ::drone_assembly_cell::action::PlacePart_FeedbackMessage feedback(::drone_assembly_cell::action::PlacePart_FeedbackMessage::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::drone_assembly_cell::action::PlacePart_FeedbackMessage msg_;
};

class Init_PlacePart_FeedbackMessage_goal_id
{
public:
  Init_PlacePart_FeedbackMessage_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PlacePart_FeedbackMessage_feedback goal_id(::drone_assembly_cell::action::PlacePart_FeedbackMessage::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_PlacePart_FeedbackMessage_feedback(msg_);
  }

private:
  ::drone_assembly_cell::action::PlacePart_FeedbackMessage msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::drone_assembly_cell::action::PlacePart_FeedbackMessage>()
{
  return drone_assembly_cell::action::builder::Init_PlacePart_FeedbackMessage_goal_id();
}

}  // namespace drone_assembly_cell

#endif  // DRONE_ASSEMBLY_CELL__ACTION__DETAIL__PLACE_PART__BUILDER_HPP_
