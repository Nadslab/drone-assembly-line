// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from drone_assembly_cell:srv/MoveToStation.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "drone_assembly_cell/srv/move_to_station.hpp"


#ifndef DRONE_ASSEMBLY_CELL__SRV__DETAIL__MOVE_TO_STATION__BUILDER_HPP_
#define DRONE_ASSEMBLY_CELL__SRV__DETAIL__MOVE_TO_STATION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "drone_assembly_cell/srv/detail/move_to_station__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace drone_assembly_cell
{

namespace srv
{

namespace builder
{

class Init_MoveToStation_Request_target_station
{
public:
  explicit Init_MoveToStation_Request_target_station(::drone_assembly_cell::srv::MoveToStation_Request & msg)
  : msg_(msg)
  {}
  ::drone_assembly_cell::srv::MoveToStation_Request target_station(::drone_assembly_cell::srv::MoveToStation_Request::_target_station_type arg)
  {
    msg_.target_station = std::move(arg);
    return std::move(msg_);
  }

private:
  ::drone_assembly_cell::srv::MoveToStation_Request msg_;
};

class Init_MoveToStation_Request_drone_id
{
public:
  Init_MoveToStation_Request_drone_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MoveToStation_Request_target_station drone_id(::drone_assembly_cell::srv::MoveToStation_Request::_drone_id_type arg)
  {
    msg_.drone_id = std::move(arg);
    return Init_MoveToStation_Request_target_station(msg_);
  }

private:
  ::drone_assembly_cell::srv::MoveToStation_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::drone_assembly_cell::srv::MoveToStation_Request>()
{
  return drone_assembly_cell::srv::builder::Init_MoveToStation_Request_drone_id();
}

}  // namespace drone_assembly_cell


namespace drone_assembly_cell
{

namespace srv
{

namespace builder
{

class Init_MoveToStation_Response_message
{
public:
  explicit Init_MoveToStation_Response_message(::drone_assembly_cell::srv::MoveToStation_Response & msg)
  : msg_(msg)
  {}
  ::drone_assembly_cell::srv::MoveToStation_Response message(::drone_assembly_cell::srv::MoveToStation_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::drone_assembly_cell::srv::MoveToStation_Response msg_;
};

class Init_MoveToStation_Response_success
{
public:
  Init_MoveToStation_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MoveToStation_Response_message success(::drone_assembly_cell::srv::MoveToStation_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_MoveToStation_Response_message(msg_);
  }

private:
  ::drone_assembly_cell::srv::MoveToStation_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::drone_assembly_cell::srv::MoveToStation_Response>()
{
  return drone_assembly_cell::srv::builder::Init_MoveToStation_Response_success();
}

}  // namespace drone_assembly_cell


namespace drone_assembly_cell
{

namespace srv
{

namespace builder
{

class Init_MoveToStation_Event_response
{
public:
  explicit Init_MoveToStation_Event_response(::drone_assembly_cell::srv::MoveToStation_Event & msg)
  : msg_(msg)
  {}
  ::drone_assembly_cell::srv::MoveToStation_Event response(::drone_assembly_cell::srv::MoveToStation_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::drone_assembly_cell::srv::MoveToStation_Event msg_;
};

class Init_MoveToStation_Event_request
{
public:
  explicit Init_MoveToStation_Event_request(::drone_assembly_cell::srv::MoveToStation_Event & msg)
  : msg_(msg)
  {}
  Init_MoveToStation_Event_response request(::drone_assembly_cell::srv::MoveToStation_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_MoveToStation_Event_response(msg_);
  }

private:
  ::drone_assembly_cell::srv::MoveToStation_Event msg_;
};

class Init_MoveToStation_Event_info
{
public:
  Init_MoveToStation_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MoveToStation_Event_request info(::drone_assembly_cell::srv::MoveToStation_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_MoveToStation_Event_request(msg_);
  }

private:
  ::drone_assembly_cell::srv::MoveToStation_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::drone_assembly_cell::srv::MoveToStation_Event>()
{
  return drone_assembly_cell::srv::builder::Init_MoveToStation_Event_info();
}

}  // namespace drone_assembly_cell

#endif  // DRONE_ASSEMBLY_CELL__SRV__DETAIL__MOVE_TO_STATION__BUILDER_HPP_
