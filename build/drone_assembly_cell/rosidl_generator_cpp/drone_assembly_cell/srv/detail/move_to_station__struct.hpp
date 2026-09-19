// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from drone_assembly_cell:srv/MoveToStation.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "drone_assembly_cell/srv/move_to_station.hpp"


#ifndef DRONE_ASSEMBLY_CELL__SRV__DETAIL__MOVE_TO_STATION__STRUCT_HPP_
#define DRONE_ASSEMBLY_CELL__SRV__DETAIL__MOVE_TO_STATION__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__drone_assembly_cell__srv__MoveToStation_Request __attribute__((deprecated))
#else
# define DEPRECATED__drone_assembly_cell__srv__MoveToStation_Request __declspec(deprecated)
#endif

namespace drone_assembly_cell
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct MoveToStation_Request_
{
  using Type = MoveToStation_Request_<ContainerAllocator>;

  explicit MoveToStation_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->drone_id = "";
      this->target_station = 0l;
    }
  }

  explicit MoveToStation_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : drone_id(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->drone_id = "";
      this->target_station = 0l;
    }
  }

  // field types and members
  using _drone_id_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _drone_id_type drone_id;
  using _target_station_type =
    int32_t;
  _target_station_type target_station;

  // setters for named parameter idiom
  Type & set__drone_id(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->drone_id = _arg;
    return *this;
  }
  Type & set__target_station(
    const int32_t & _arg)
  {
    this->target_station = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    drone_assembly_cell::srv::MoveToStation_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const drone_assembly_cell::srv::MoveToStation_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<drone_assembly_cell::srv::MoveToStation_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<drone_assembly_cell::srv::MoveToStation_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      drone_assembly_cell::srv::MoveToStation_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<drone_assembly_cell::srv::MoveToStation_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      drone_assembly_cell::srv::MoveToStation_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<drone_assembly_cell::srv::MoveToStation_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<drone_assembly_cell::srv::MoveToStation_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<drone_assembly_cell::srv::MoveToStation_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__drone_assembly_cell__srv__MoveToStation_Request
    std::shared_ptr<drone_assembly_cell::srv::MoveToStation_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__drone_assembly_cell__srv__MoveToStation_Request
    std::shared_ptr<drone_assembly_cell::srv::MoveToStation_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MoveToStation_Request_ & other) const
  {
    if (this->drone_id != other.drone_id) {
      return false;
    }
    if (this->target_station != other.target_station) {
      return false;
    }
    return true;
  }
  bool operator!=(const MoveToStation_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MoveToStation_Request_

// alias to use template instance with default allocator
using MoveToStation_Request =
  drone_assembly_cell::srv::MoveToStation_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace drone_assembly_cell


#ifndef _WIN32
# define DEPRECATED__drone_assembly_cell__srv__MoveToStation_Response __attribute__((deprecated))
#else
# define DEPRECATED__drone_assembly_cell__srv__MoveToStation_Response __declspec(deprecated)
#endif

namespace drone_assembly_cell
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct MoveToStation_Response_
{
  using Type = MoveToStation_Response_<ContainerAllocator>;

  explicit MoveToStation_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  explicit MoveToStation_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : message(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;
  using _message_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _message_type message;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }
  Type & set__message(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->message = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    drone_assembly_cell::srv::MoveToStation_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const drone_assembly_cell::srv::MoveToStation_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<drone_assembly_cell::srv::MoveToStation_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<drone_assembly_cell::srv::MoveToStation_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      drone_assembly_cell::srv::MoveToStation_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<drone_assembly_cell::srv::MoveToStation_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      drone_assembly_cell::srv::MoveToStation_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<drone_assembly_cell::srv::MoveToStation_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<drone_assembly_cell::srv::MoveToStation_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<drone_assembly_cell::srv::MoveToStation_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__drone_assembly_cell__srv__MoveToStation_Response
    std::shared_ptr<drone_assembly_cell::srv::MoveToStation_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__drone_assembly_cell__srv__MoveToStation_Response
    std::shared_ptr<drone_assembly_cell::srv::MoveToStation_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MoveToStation_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    return true;
  }
  bool operator!=(const MoveToStation_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MoveToStation_Response_

// alias to use template instance with default allocator
using MoveToStation_Response =
  drone_assembly_cell::srv::MoveToStation_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace drone_assembly_cell


// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__drone_assembly_cell__srv__MoveToStation_Event __attribute__((deprecated))
#else
# define DEPRECATED__drone_assembly_cell__srv__MoveToStation_Event __declspec(deprecated)
#endif

namespace drone_assembly_cell
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct MoveToStation_Event_
{
  using Type = MoveToStation_Event_<ContainerAllocator>;

  explicit MoveToStation_Event_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_init)
  {
    (void)_init;
  }

  explicit MoveToStation_Event_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _info_type =
    service_msgs::msg::ServiceEventInfo_<ContainerAllocator>;
  _info_type info;
  using _request_type =
    rosidl_runtime_cpp::BoundedVector<drone_assembly_cell::srv::MoveToStation_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<drone_assembly_cell::srv::MoveToStation_Request_<ContainerAllocator>>>;
  _request_type request;
  using _response_type =
    rosidl_runtime_cpp::BoundedVector<drone_assembly_cell::srv::MoveToStation_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<drone_assembly_cell::srv::MoveToStation_Response_<ContainerAllocator>>>;
  _response_type response;

  // setters for named parameter idiom
  Type & set__info(
    const service_msgs::msg::ServiceEventInfo_<ContainerAllocator> & _arg)
  {
    this->info = _arg;
    return *this;
  }
  Type & set__request(
    const rosidl_runtime_cpp::BoundedVector<drone_assembly_cell::srv::MoveToStation_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<drone_assembly_cell::srv::MoveToStation_Request_<ContainerAllocator>>> & _arg)
  {
    this->request = _arg;
    return *this;
  }
  Type & set__response(
    const rosidl_runtime_cpp::BoundedVector<drone_assembly_cell::srv::MoveToStation_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<drone_assembly_cell::srv::MoveToStation_Response_<ContainerAllocator>>> & _arg)
  {
    this->response = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    drone_assembly_cell::srv::MoveToStation_Event_<ContainerAllocator> *;
  using ConstRawPtr =
    const drone_assembly_cell::srv::MoveToStation_Event_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<drone_assembly_cell::srv::MoveToStation_Event_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<drone_assembly_cell::srv::MoveToStation_Event_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      drone_assembly_cell::srv::MoveToStation_Event_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<drone_assembly_cell::srv::MoveToStation_Event_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      drone_assembly_cell::srv::MoveToStation_Event_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<drone_assembly_cell::srv::MoveToStation_Event_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<drone_assembly_cell::srv::MoveToStation_Event_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<drone_assembly_cell::srv::MoveToStation_Event_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__drone_assembly_cell__srv__MoveToStation_Event
    std::shared_ptr<drone_assembly_cell::srv::MoveToStation_Event_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__drone_assembly_cell__srv__MoveToStation_Event
    std::shared_ptr<drone_assembly_cell::srv::MoveToStation_Event_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MoveToStation_Event_ & other) const
  {
    if (this->info != other.info) {
      return false;
    }
    if (this->request != other.request) {
      return false;
    }
    if (this->response != other.response) {
      return false;
    }
    return true;
  }
  bool operator!=(const MoveToStation_Event_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MoveToStation_Event_

// alias to use template instance with default allocator
using MoveToStation_Event =
  drone_assembly_cell::srv::MoveToStation_Event_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace drone_assembly_cell

namespace drone_assembly_cell
{

namespace srv
{

struct MoveToStation
{
  using Request = drone_assembly_cell::srv::MoveToStation_Request;
  using Response = drone_assembly_cell::srv::MoveToStation_Response;
  using Event = drone_assembly_cell::srv::MoveToStation_Event;
};

}  // namespace srv

}  // namespace drone_assembly_cell

#endif  // DRONE_ASSEMBLY_CELL__SRV__DETAIL__MOVE_TO_STATION__STRUCT_HPP_
