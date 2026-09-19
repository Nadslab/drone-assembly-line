// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from drone_assembly_cell:action/PickPart.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "drone_assembly_cell/action/pick_part.hpp"


#ifndef DRONE_ASSEMBLY_CELL__ACTION__DETAIL__PICK_PART__STRUCT_HPP_
#define DRONE_ASSEMBLY_CELL__ACTION__DETAIL__PICK_PART__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'source_pose'
#include "geometry_msgs/msg/detail/pose_stamped__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__drone_assembly_cell__action__PickPart_Goal __attribute__((deprecated))
#else
# define DEPRECATED__drone_assembly_cell__action__PickPart_Goal __declspec(deprecated)
#endif

namespace drone_assembly_cell
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct PickPart_Goal_
{
  using Type = PickPart_Goal_<ContainerAllocator>;

  explicit PickPart_Goal_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : source_pose(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->approach_height = 0.0;
      this->gripper_close_pos = 0.0;
    }
  }

  explicit PickPart_Goal_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : source_pose(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->approach_height = 0.0;
      this->gripper_close_pos = 0.0;
    }
  }

  // field types and members
  using _source_pose_type =
    geometry_msgs::msg::PoseStamped_<ContainerAllocator>;
  _source_pose_type source_pose;
  using _approach_height_type =
    double;
  _approach_height_type approach_height;
  using _gripper_close_pos_type =
    double;
  _gripper_close_pos_type gripper_close_pos;

  // setters for named parameter idiom
  Type & set__source_pose(
    const geometry_msgs::msg::PoseStamped_<ContainerAllocator> & _arg)
  {
    this->source_pose = _arg;
    return *this;
  }
  Type & set__approach_height(
    const double & _arg)
  {
    this->approach_height = _arg;
    return *this;
  }
  Type & set__gripper_close_pos(
    const double & _arg)
  {
    this->gripper_close_pos = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    drone_assembly_cell::action::PickPart_Goal_<ContainerAllocator> *;
  using ConstRawPtr =
    const drone_assembly_cell::action::PickPart_Goal_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<drone_assembly_cell::action::PickPart_Goal_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<drone_assembly_cell::action::PickPart_Goal_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      drone_assembly_cell::action::PickPart_Goal_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<drone_assembly_cell::action::PickPart_Goal_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      drone_assembly_cell::action::PickPart_Goal_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<drone_assembly_cell::action::PickPart_Goal_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<drone_assembly_cell::action::PickPart_Goal_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<drone_assembly_cell::action::PickPart_Goal_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__drone_assembly_cell__action__PickPart_Goal
    std::shared_ptr<drone_assembly_cell::action::PickPart_Goal_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__drone_assembly_cell__action__PickPart_Goal
    std::shared_ptr<drone_assembly_cell::action::PickPart_Goal_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PickPart_Goal_ & other) const
  {
    if (this->source_pose != other.source_pose) {
      return false;
    }
    if (this->approach_height != other.approach_height) {
      return false;
    }
    if (this->gripper_close_pos != other.gripper_close_pos) {
      return false;
    }
    return true;
  }
  bool operator!=(const PickPart_Goal_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PickPart_Goal_

// alias to use template instance with default allocator
using PickPart_Goal =
  drone_assembly_cell::action::PickPart_Goal_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace drone_assembly_cell


#ifndef _WIN32
# define DEPRECATED__drone_assembly_cell__action__PickPart_Result __attribute__((deprecated))
#else
# define DEPRECATED__drone_assembly_cell__action__PickPart_Result __declspec(deprecated)
#endif

namespace drone_assembly_cell
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct PickPart_Result_
{
  using Type = PickPart_Result_<ContainerAllocator>;

  explicit PickPart_Result_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  explicit PickPart_Result_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    drone_assembly_cell::action::PickPart_Result_<ContainerAllocator> *;
  using ConstRawPtr =
    const drone_assembly_cell::action::PickPart_Result_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<drone_assembly_cell::action::PickPart_Result_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<drone_assembly_cell::action::PickPart_Result_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      drone_assembly_cell::action::PickPart_Result_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<drone_assembly_cell::action::PickPart_Result_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      drone_assembly_cell::action::PickPart_Result_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<drone_assembly_cell::action::PickPart_Result_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<drone_assembly_cell::action::PickPart_Result_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<drone_assembly_cell::action::PickPart_Result_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__drone_assembly_cell__action__PickPart_Result
    std::shared_ptr<drone_assembly_cell::action::PickPart_Result_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__drone_assembly_cell__action__PickPart_Result
    std::shared_ptr<drone_assembly_cell::action::PickPart_Result_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PickPart_Result_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    return true;
  }
  bool operator!=(const PickPart_Result_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PickPart_Result_

// alias to use template instance with default allocator
using PickPart_Result =
  drone_assembly_cell::action::PickPart_Result_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace drone_assembly_cell


#ifndef _WIN32
# define DEPRECATED__drone_assembly_cell__action__PickPart_Feedback __attribute__((deprecated))
#else
# define DEPRECATED__drone_assembly_cell__action__PickPart_Feedback __declspec(deprecated)
#endif

namespace drone_assembly_cell
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct PickPart_Feedback_
{
  using Type = PickPart_Feedback_<ContainerAllocator>;

  explicit PickPart_Feedback_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->phase = "";
    }
  }

  explicit PickPart_Feedback_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : phase(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->phase = "";
    }
  }

  // field types and members
  using _phase_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _phase_type phase;

  // setters for named parameter idiom
  Type & set__phase(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->phase = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    drone_assembly_cell::action::PickPart_Feedback_<ContainerAllocator> *;
  using ConstRawPtr =
    const drone_assembly_cell::action::PickPart_Feedback_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<drone_assembly_cell::action::PickPart_Feedback_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<drone_assembly_cell::action::PickPart_Feedback_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      drone_assembly_cell::action::PickPart_Feedback_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<drone_assembly_cell::action::PickPart_Feedback_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      drone_assembly_cell::action::PickPart_Feedback_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<drone_assembly_cell::action::PickPart_Feedback_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<drone_assembly_cell::action::PickPart_Feedback_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<drone_assembly_cell::action::PickPart_Feedback_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__drone_assembly_cell__action__PickPart_Feedback
    std::shared_ptr<drone_assembly_cell::action::PickPart_Feedback_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__drone_assembly_cell__action__PickPart_Feedback
    std::shared_ptr<drone_assembly_cell::action::PickPart_Feedback_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PickPart_Feedback_ & other) const
  {
    if (this->phase != other.phase) {
      return false;
    }
    return true;
  }
  bool operator!=(const PickPart_Feedback_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PickPart_Feedback_

// alias to use template instance with default allocator
using PickPart_Feedback =
  drone_assembly_cell::action::PickPart_Feedback_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace drone_assembly_cell


// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"
// Member 'goal'
#include "drone_assembly_cell/action/detail/pick_part__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__drone_assembly_cell__action__PickPart_SendGoal_Request __attribute__((deprecated))
#else
# define DEPRECATED__drone_assembly_cell__action__PickPart_SendGoal_Request __declspec(deprecated)
#endif

namespace drone_assembly_cell
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct PickPart_SendGoal_Request_
{
  using Type = PickPart_SendGoal_Request_<ContainerAllocator>;

  explicit PickPart_SendGoal_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init),
    goal(_init)
  {
    (void)_init;
  }

  explicit PickPart_SendGoal_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_alloc, _init),
    goal(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _goal_id_type =
    unique_identifier_msgs::msg::UUID_<ContainerAllocator>;
  _goal_id_type goal_id;
  using _goal_type =
    drone_assembly_cell::action::PickPart_Goal_<ContainerAllocator>;
  _goal_type goal;

  // setters for named parameter idiom
  Type & set__goal_id(
    const unique_identifier_msgs::msg::UUID_<ContainerAllocator> & _arg)
  {
    this->goal_id = _arg;
    return *this;
  }
  Type & set__goal(
    const drone_assembly_cell::action::PickPart_Goal_<ContainerAllocator> & _arg)
  {
    this->goal = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    drone_assembly_cell::action::PickPart_SendGoal_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const drone_assembly_cell::action::PickPart_SendGoal_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<drone_assembly_cell::action::PickPart_SendGoal_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<drone_assembly_cell::action::PickPart_SendGoal_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      drone_assembly_cell::action::PickPart_SendGoal_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<drone_assembly_cell::action::PickPart_SendGoal_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      drone_assembly_cell::action::PickPart_SendGoal_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<drone_assembly_cell::action::PickPart_SendGoal_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<drone_assembly_cell::action::PickPart_SendGoal_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<drone_assembly_cell::action::PickPart_SendGoal_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__drone_assembly_cell__action__PickPart_SendGoal_Request
    std::shared_ptr<drone_assembly_cell::action::PickPart_SendGoal_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__drone_assembly_cell__action__PickPart_SendGoal_Request
    std::shared_ptr<drone_assembly_cell::action::PickPart_SendGoal_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PickPart_SendGoal_Request_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    if (this->goal != other.goal) {
      return false;
    }
    return true;
  }
  bool operator!=(const PickPart_SendGoal_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PickPart_SendGoal_Request_

// alias to use template instance with default allocator
using PickPart_SendGoal_Request =
  drone_assembly_cell::action::PickPart_SendGoal_Request_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace drone_assembly_cell


// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__drone_assembly_cell__action__PickPart_SendGoal_Response __attribute__((deprecated))
#else
# define DEPRECATED__drone_assembly_cell__action__PickPart_SendGoal_Response __declspec(deprecated)
#endif

namespace drone_assembly_cell
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct PickPart_SendGoal_Response_
{
  using Type = PickPart_SendGoal_Response_<ContainerAllocator>;

  explicit PickPart_SendGoal_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->accepted = false;
    }
  }

  explicit PickPart_SendGoal_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->accepted = false;
    }
  }

  // field types and members
  using _accepted_type =
    bool;
  _accepted_type accepted;
  using _stamp_type =
    builtin_interfaces::msg::Time_<ContainerAllocator>;
  _stamp_type stamp;

  // setters for named parameter idiom
  Type & set__accepted(
    const bool & _arg)
  {
    this->accepted = _arg;
    return *this;
  }
  Type & set__stamp(
    const builtin_interfaces::msg::Time_<ContainerAllocator> & _arg)
  {
    this->stamp = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    drone_assembly_cell::action::PickPart_SendGoal_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const drone_assembly_cell::action::PickPart_SendGoal_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<drone_assembly_cell::action::PickPart_SendGoal_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<drone_assembly_cell::action::PickPart_SendGoal_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      drone_assembly_cell::action::PickPart_SendGoal_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<drone_assembly_cell::action::PickPart_SendGoal_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      drone_assembly_cell::action::PickPart_SendGoal_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<drone_assembly_cell::action::PickPart_SendGoal_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<drone_assembly_cell::action::PickPart_SendGoal_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<drone_assembly_cell::action::PickPart_SendGoal_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__drone_assembly_cell__action__PickPart_SendGoal_Response
    std::shared_ptr<drone_assembly_cell::action::PickPart_SendGoal_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__drone_assembly_cell__action__PickPart_SendGoal_Response
    std::shared_ptr<drone_assembly_cell::action::PickPart_SendGoal_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PickPart_SendGoal_Response_ & other) const
  {
    if (this->accepted != other.accepted) {
      return false;
    }
    if (this->stamp != other.stamp) {
      return false;
    }
    return true;
  }
  bool operator!=(const PickPart_SendGoal_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PickPart_SendGoal_Response_

// alias to use template instance with default allocator
using PickPart_SendGoal_Response =
  drone_assembly_cell::action::PickPart_SendGoal_Response_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace drone_assembly_cell


// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__drone_assembly_cell__action__PickPart_SendGoal_Event __attribute__((deprecated))
#else
# define DEPRECATED__drone_assembly_cell__action__PickPart_SendGoal_Event __declspec(deprecated)
#endif

namespace drone_assembly_cell
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct PickPart_SendGoal_Event_
{
  using Type = PickPart_SendGoal_Event_<ContainerAllocator>;

  explicit PickPart_SendGoal_Event_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_init)
  {
    (void)_init;
  }

  explicit PickPart_SendGoal_Event_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _info_type =
    service_msgs::msg::ServiceEventInfo_<ContainerAllocator>;
  _info_type info;
  using _request_type =
    rosidl_runtime_cpp::BoundedVector<drone_assembly_cell::action::PickPart_SendGoal_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<drone_assembly_cell::action::PickPart_SendGoal_Request_<ContainerAllocator>>>;
  _request_type request;
  using _response_type =
    rosidl_runtime_cpp::BoundedVector<drone_assembly_cell::action::PickPart_SendGoal_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<drone_assembly_cell::action::PickPart_SendGoal_Response_<ContainerAllocator>>>;
  _response_type response;

  // setters for named parameter idiom
  Type & set__info(
    const service_msgs::msg::ServiceEventInfo_<ContainerAllocator> & _arg)
  {
    this->info = _arg;
    return *this;
  }
  Type & set__request(
    const rosidl_runtime_cpp::BoundedVector<drone_assembly_cell::action::PickPart_SendGoal_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<drone_assembly_cell::action::PickPart_SendGoal_Request_<ContainerAllocator>>> & _arg)
  {
    this->request = _arg;
    return *this;
  }
  Type & set__response(
    const rosidl_runtime_cpp::BoundedVector<drone_assembly_cell::action::PickPart_SendGoal_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<drone_assembly_cell::action::PickPart_SendGoal_Response_<ContainerAllocator>>> & _arg)
  {
    this->response = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    drone_assembly_cell::action::PickPart_SendGoal_Event_<ContainerAllocator> *;
  using ConstRawPtr =
    const drone_assembly_cell::action::PickPart_SendGoal_Event_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<drone_assembly_cell::action::PickPart_SendGoal_Event_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<drone_assembly_cell::action::PickPart_SendGoal_Event_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      drone_assembly_cell::action::PickPart_SendGoal_Event_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<drone_assembly_cell::action::PickPart_SendGoal_Event_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      drone_assembly_cell::action::PickPart_SendGoal_Event_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<drone_assembly_cell::action::PickPart_SendGoal_Event_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<drone_assembly_cell::action::PickPart_SendGoal_Event_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<drone_assembly_cell::action::PickPart_SendGoal_Event_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__drone_assembly_cell__action__PickPart_SendGoal_Event
    std::shared_ptr<drone_assembly_cell::action::PickPart_SendGoal_Event_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__drone_assembly_cell__action__PickPart_SendGoal_Event
    std::shared_ptr<drone_assembly_cell::action::PickPart_SendGoal_Event_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PickPart_SendGoal_Event_ & other) const
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
  bool operator!=(const PickPart_SendGoal_Event_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PickPart_SendGoal_Event_

// alias to use template instance with default allocator
using PickPart_SendGoal_Event =
  drone_assembly_cell::action::PickPart_SendGoal_Event_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace drone_assembly_cell

namespace drone_assembly_cell
{

namespace action
{

struct PickPart_SendGoal
{
  using Request = drone_assembly_cell::action::PickPart_SendGoal_Request;
  using Response = drone_assembly_cell::action::PickPart_SendGoal_Response;
  using Event = drone_assembly_cell::action::PickPart_SendGoal_Event;
};

}  // namespace action

}  // namespace drone_assembly_cell


// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__drone_assembly_cell__action__PickPart_GetResult_Request __attribute__((deprecated))
#else
# define DEPRECATED__drone_assembly_cell__action__PickPart_GetResult_Request __declspec(deprecated)
#endif

namespace drone_assembly_cell
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct PickPart_GetResult_Request_
{
  using Type = PickPart_GetResult_Request_<ContainerAllocator>;

  explicit PickPart_GetResult_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init)
  {
    (void)_init;
  }

  explicit PickPart_GetResult_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _goal_id_type =
    unique_identifier_msgs::msg::UUID_<ContainerAllocator>;
  _goal_id_type goal_id;

  // setters for named parameter idiom
  Type & set__goal_id(
    const unique_identifier_msgs::msg::UUID_<ContainerAllocator> & _arg)
  {
    this->goal_id = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    drone_assembly_cell::action::PickPart_GetResult_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const drone_assembly_cell::action::PickPart_GetResult_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<drone_assembly_cell::action::PickPart_GetResult_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<drone_assembly_cell::action::PickPart_GetResult_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      drone_assembly_cell::action::PickPart_GetResult_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<drone_assembly_cell::action::PickPart_GetResult_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      drone_assembly_cell::action::PickPart_GetResult_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<drone_assembly_cell::action::PickPart_GetResult_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<drone_assembly_cell::action::PickPart_GetResult_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<drone_assembly_cell::action::PickPart_GetResult_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__drone_assembly_cell__action__PickPart_GetResult_Request
    std::shared_ptr<drone_assembly_cell::action::PickPart_GetResult_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__drone_assembly_cell__action__PickPart_GetResult_Request
    std::shared_ptr<drone_assembly_cell::action::PickPart_GetResult_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PickPart_GetResult_Request_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    return true;
  }
  bool operator!=(const PickPart_GetResult_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PickPart_GetResult_Request_

// alias to use template instance with default allocator
using PickPart_GetResult_Request =
  drone_assembly_cell::action::PickPart_GetResult_Request_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace drone_assembly_cell


// Include directives for member types
// Member 'result'
// already included above
// #include "drone_assembly_cell/action/detail/pick_part__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__drone_assembly_cell__action__PickPart_GetResult_Response __attribute__((deprecated))
#else
# define DEPRECATED__drone_assembly_cell__action__PickPart_GetResult_Response __declspec(deprecated)
#endif

namespace drone_assembly_cell
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct PickPart_GetResult_Response_
{
  using Type = PickPart_GetResult_Response_<ContainerAllocator>;

  explicit PickPart_GetResult_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : result(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = 0;
    }
  }

  explicit PickPart_GetResult_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : result(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = 0;
    }
  }

  // field types and members
  using _status_type =
    int8_t;
  _status_type status;
  using _result_type =
    drone_assembly_cell::action::PickPart_Result_<ContainerAllocator>;
  _result_type result;

  // setters for named parameter idiom
  Type & set__status(
    const int8_t & _arg)
  {
    this->status = _arg;
    return *this;
  }
  Type & set__result(
    const drone_assembly_cell::action::PickPart_Result_<ContainerAllocator> & _arg)
  {
    this->result = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    drone_assembly_cell::action::PickPart_GetResult_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const drone_assembly_cell::action::PickPart_GetResult_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<drone_assembly_cell::action::PickPart_GetResult_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<drone_assembly_cell::action::PickPart_GetResult_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      drone_assembly_cell::action::PickPart_GetResult_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<drone_assembly_cell::action::PickPart_GetResult_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      drone_assembly_cell::action::PickPart_GetResult_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<drone_assembly_cell::action::PickPart_GetResult_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<drone_assembly_cell::action::PickPart_GetResult_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<drone_assembly_cell::action::PickPart_GetResult_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__drone_assembly_cell__action__PickPart_GetResult_Response
    std::shared_ptr<drone_assembly_cell::action::PickPart_GetResult_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__drone_assembly_cell__action__PickPart_GetResult_Response
    std::shared_ptr<drone_assembly_cell::action::PickPart_GetResult_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PickPart_GetResult_Response_ & other) const
  {
    if (this->status != other.status) {
      return false;
    }
    if (this->result != other.result) {
      return false;
    }
    return true;
  }
  bool operator!=(const PickPart_GetResult_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PickPart_GetResult_Response_

// alias to use template instance with default allocator
using PickPart_GetResult_Response =
  drone_assembly_cell::action::PickPart_GetResult_Response_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace drone_assembly_cell


// Include directives for member types
// Member 'info'
// already included above
// #include "service_msgs/msg/detail/service_event_info__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__drone_assembly_cell__action__PickPart_GetResult_Event __attribute__((deprecated))
#else
# define DEPRECATED__drone_assembly_cell__action__PickPart_GetResult_Event __declspec(deprecated)
#endif

namespace drone_assembly_cell
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct PickPart_GetResult_Event_
{
  using Type = PickPart_GetResult_Event_<ContainerAllocator>;

  explicit PickPart_GetResult_Event_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_init)
  {
    (void)_init;
  }

  explicit PickPart_GetResult_Event_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _info_type =
    service_msgs::msg::ServiceEventInfo_<ContainerAllocator>;
  _info_type info;
  using _request_type =
    rosidl_runtime_cpp::BoundedVector<drone_assembly_cell::action::PickPart_GetResult_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<drone_assembly_cell::action::PickPart_GetResult_Request_<ContainerAllocator>>>;
  _request_type request;
  using _response_type =
    rosidl_runtime_cpp::BoundedVector<drone_assembly_cell::action::PickPart_GetResult_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<drone_assembly_cell::action::PickPart_GetResult_Response_<ContainerAllocator>>>;
  _response_type response;

  // setters for named parameter idiom
  Type & set__info(
    const service_msgs::msg::ServiceEventInfo_<ContainerAllocator> & _arg)
  {
    this->info = _arg;
    return *this;
  }
  Type & set__request(
    const rosidl_runtime_cpp::BoundedVector<drone_assembly_cell::action::PickPart_GetResult_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<drone_assembly_cell::action::PickPart_GetResult_Request_<ContainerAllocator>>> & _arg)
  {
    this->request = _arg;
    return *this;
  }
  Type & set__response(
    const rosidl_runtime_cpp::BoundedVector<drone_assembly_cell::action::PickPart_GetResult_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<drone_assembly_cell::action::PickPart_GetResult_Response_<ContainerAllocator>>> & _arg)
  {
    this->response = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    drone_assembly_cell::action::PickPart_GetResult_Event_<ContainerAllocator> *;
  using ConstRawPtr =
    const drone_assembly_cell::action::PickPart_GetResult_Event_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<drone_assembly_cell::action::PickPart_GetResult_Event_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<drone_assembly_cell::action::PickPart_GetResult_Event_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      drone_assembly_cell::action::PickPart_GetResult_Event_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<drone_assembly_cell::action::PickPart_GetResult_Event_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      drone_assembly_cell::action::PickPart_GetResult_Event_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<drone_assembly_cell::action::PickPart_GetResult_Event_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<drone_assembly_cell::action::PickPart_GetResult_Event_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<drone_assembly_cell::action::PickPart_GetResult_Event_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__drone_assembly_cell__action__PickPart_GetResult_Event
    std::shared_ptr<drone_assembly_cell::action::PickPart_GetResult_Event_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__drone_assembly_cell__action__PickPart_GetResult_Event
    std::shared_ptr<drone_assembly_cell::action::PickPart_GetResult_Event_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PickPart_GetResult_Event_ & other) const
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
  bool operator!=(const PickPart_GetResult_Event_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PickPart_GetResult_Event_

// alias to use template instance with default allocator
using PickPart_GetResult_Event =
  drone_assembly_cell::action::PickPart_GetResult_Event_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace drone_assembly_cell

namespace drone_assembly_cell
{

namespace action
{

struct PickPart_GetResult
{
  using Request = drone_assembly_cell::action::PickPart_GetResult_Request;
  using Response = drone_assembly_cell::action::PickPart_GetResult_Response;
  using Event = drone_assembly_cell::action::PickPart_GetResult_Event;
};

}  // namespace action

}  // namespace drone_assembly_cell


// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"
// Member 'feedback'
// already included above
// #include "drone_assembly_cell/action/detail/pick_part__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__drone_assembly_cell__action__PickPart_FeedbackMessage __attribute__((deprecated))
#else
# define DEPRECATED__drone_assembly_cell__action__PickPart_FeedbackMessage __declspec(deprecated)
#endif

namespace drone_assembly_cell
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct PickPart_FeedbackMessage_
{
  using Type = PickPart_FeedbackMessage_<ContainerAllocator>;

  explicit PickPart_FeedbackMessage_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init),
    feedback(_init)
  {
    (void)_init;
  }

  explicit PickPart_FeedbackMessage_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_alloc, _init),
    feedback(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _goal_id_type =
    unique_identifier_msgs::msg::UUID_<ContainerAllocator>;
  _goal_id_type goal_id;
  using _feedback_type =
    drone_assembly_cell::action::PickPart_Feedback_<ContainerAllocator>;
  _feedback_type feedback;

  // setters for named parameter idiom
  Type & set__goal_id(
    const unique_identifier_msgs::msg::UUID_<ContainerAllocator> & _arg)
  {
    this->goal_id = _arg;
    return *this;
  }
  Type & set__feedback(
    const drone_assembly_cell::action::PickPart_Feedback_<ContainerAllocator> & _arg)
  {
    this->feedback = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    drone_assembly_cell::action::PickPart_FeedbackMessage_<ContainerAllocator> *;
  using ConstRawPtr =
    const drone_assembly_cell::action::PickPart_FeedbackMessage_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<drone_assembly_cell::action::PickPart_FeedbackMessage_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<drone_assembly_cell::action::PickPart_FeedbackMessage_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      drone_assembly_cell::action::PickPart_FeedbackMessage_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<drone_assembly_cell::action::PickPart_FeedbackMessage_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      drone_assembly_cell::action::PickPart_FeedbackMessage_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<drone_assembly_cell::action::PickPart_FeedbackMessage_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<drone_assembly_cell::action::PickPart_FeedbackMessage_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<drone_assembly_cell::action::PickPart_FeedbackMessage_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__drone_assembly_cell__action__PickPart_FeedbackMessage
    std::shared_ptr<drone_assembly_cell::action::PickPart_FeedbackMessage_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__drone_assembly_cell__action__PickPart_FeedbackMessage
    std::shared_ptr<drone_assembly_cell::action::PickPart_FeedbackMessage_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PickPart_FeedbackMessage_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    if (this->feedback != other.feedback) {
      return false;
    }
    return true;
  }
  bool operator!=(const PickPart_FeedbackMessage_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PickPart_FeedbackMessage_

// alias to use template instance with default allocator
using PickPart_FeedbackMessage =
  drone_assembly_cell::action::PickPart_FeedbackMessage_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace drone_assembly_cell

#include "action_msgs/srv/cancel_goal.hpp"
#include "action_msgs/msg/goal_info.hpp"
#include "action_msgs/msg/goal_status_array.hpp"

namespace drone_assembly_cell
{

namespace action
{

struct PickPart
{
  /// The goal message defined in the action definition.
  using Goal = drone_assembly_cell::action::PickPart_Goal;
  /// The result message defined in the action definition.
  using Result = drone_assembly_cell::action::PickPart_Result;
  /// The feedback message defined in the action definition.
  using Feedback = drone_assembly_cell::action::PickPart_Feedback;

  struct Impl
  {
    /// The send_goal service using a wrapped version of the goal message as a request.
    using SendGoalService = drone_assembly_cell::action::PickPart_SendGoal;
    /// The get_result service using a wrapped version of the result message as a response.
    using GetResultService = drone_assembly_cell::action::PickPart_GetResult;
    /// The feedback message with generic fields which wraps the feedback message.
    using FeedbackMessage = drone_assembly_cell::action::PickPart_FeedbackMessage;

    /// The generic service to cancel a goal.
    using CancelGoalService = action_msgs::srv::CancelGoal;
    /// The generic message for the status of a goal.
    using GoalStatusMessage = action_msgs::msg::GoalStatusArray;
  };
};

typedef struct PickPart PickPart;

}  // namespace action

}  // namespace drone_assembly_cell

#endif  // DRONE_ASSEMBLY_CELL__ACTION__DETAIL__PICK_PART__STRUCT_HPP_
