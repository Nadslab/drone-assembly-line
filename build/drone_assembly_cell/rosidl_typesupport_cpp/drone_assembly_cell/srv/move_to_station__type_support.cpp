// generated from rosidl_typesupport_cpp/resource/idl__type_support.cpp.em
// with input from drone_assembly_cell:srv/MoveToStation.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "drone_assembly_cell/srv/detail/move_to_station__functions.h"
#include "drone_assembly_cell/srv/detail/move_to_station__struct.hpp"
#include "rosidl_typesupport_cpp/identifier.hpp"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
#include "rosidl_typesupport_cpp/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace drone_assembly_cell
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _MoveToStation_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _MoveToStation_Request_type_support_ids_t;

static const _MoveToStation_Request_type_support_ids_t _MoveToStation_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _MoveToStation_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _MoveToStation_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _MoveToStation_Request_type_support_symbol_names_t _MoveToStation_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, drone_assembly_cell, srv, MoveToStation_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, drone_assembly_cell, srv, MoveToStation_Request)),
  }
};

typedef struct _MoveToStation_Request_type_support_data_t
{
  void * data[2];
} _MoveToStation_Request_type_support_data_t;

static _MoveToStation_Request_type_support_data_t _MoveToStation_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _MoveToStation_Request_message_typesupport_map = {
  2,
  "drone_assembly_cell",
  &_MoveToStation_Request_message_typesupport_ids.typesupport_identifier[0],
  &_MoveToStation_Request_message_typesupport_symbol_names.symbol_name[0],
  &_MoveToStation_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t MoveToStation_Request_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_MoveToStation_Request_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &drone_assembly_cell__srv__MoveToStation_Request__get_type_hash,
  &drone_assembly_cell__srv__MoveToStation_Request__get_type_description,
  &drone_assembly_cell__srv__MoveToStation_Request__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace drone_assembly_cell

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<drone_assembly_cell::srv::MoveToStation_Request>()
{
  return &::drone_assembly_cell::srv::rosidl_typesupport_cpp::MoveToStation_Request_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, drone_assembly_cell, srv, MoveToStation_Request)() {
  return get_message_type_support_handle<drone_assembly_cell::srv::MoveToStation_Request>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "drone_assembly_cell/srv/detail/move_to_station__functions.h"
// already included above
// #include "drone_assembly_cell/srv/detail/move_to_station__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace drone_assembly_cell
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _MoveToStation_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _MoveToStation_Response_type_support_ids_t;

static const _MoveToStation_Response_type_support_ids_t _MoveToStation_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _MoveToStation_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _MoveToStation_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _MoveToStation_Response_type_support_symbol_names_t _MoveToStation_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, drone_assembly_cell, srv, MoveToStation_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, drone_assembly_cell, srv, MoveToStation_Response)),
  }
};

typedef struct _MoveToStation_Response_type_support_data_t
{
  void * data[2];
} _MoveToStation_Response_type_support_data_t;

static _MoveToStation_Response_type_support_data_t _MoveToStation_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _MoveToStation_Response_message_typesupport_map = {
  2,
  "drone_assembly_cell",
  &_MoveToStation_Response_message_typesupport_ids.typesupport_identifier[0],
  &_MoveToStation_Response_message_typesupport_symbol_names.symbol_name[0],
  &_MoveToStation_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t MoveToStation_Response_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_MoveToStation_Response_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &drone_assembly_cell__srv__MoveToStation_Response__get_type_hash,
  &drone_assembly_cell__srv__MoveToStation_Response__get_type_description,
  &drone_assembly_cell__srv__MoveToStation_Response__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace drone_assembly_cell

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<drone_assembly_cell::srv::MoveToStation_Response>()
{
  return &::drone_assembly_cell::srv::rosidl_typesupport_cpp::MoveToStation_Response_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, drone_assembly_cell, srv, MoveToStation_Response)() {
  return get_message_type_support_handle<drone_assembly_cell::srv::MoveToStation_Response>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "drone_assembly_cell/srv/detail/move_to_station__functions.h"
// already included above
// #include "drone_assembly_cell/srv/detail/move_to_station__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace drone_assembly_cell
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _MoveToStation_Event_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _MoveToStation_Event_type_support_ids_t;

static const _MoveToStation_Event_type_support_ids_t _MoveToStation_Event_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _MoveToStation_Event_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _MoveToStation_Event_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _MoveToStation_Event_type_support_symbol_names_t _MoveToStation_Event_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, drone_assembly_cell, srv, MoveToStation_Event)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, drone_assembly_cell, srv, MoveToStation_Event)),
  }
};

typedef struct _MoveToStation_Event_type_support_data_t
{
  void * data[2];
} _MoveToStation_Event_type_support_data_t;

static _MoveToStation_Event_type_support_data_t _MoveToStation_Event_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _MoveToStation_Event_message_typesupport_map = {
  2,
  "drone_assembly_cell",
  &_MoveToStation_Event_message_typesupport_ids.typesupport_identifier[0],
  &_MoveToStation_Event_message_typesupport_symbol_names.symbol_name[0],
  &_MoveToStation_Event_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t MoveToStation_Event_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_MoveToStation_Event_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &drone_assembly_cell__srv__MoveToStation_Event__get_type_hash,
  &drone_assembly_cell__srv__MoveToStation_Event__get_type_description,
  &drone_assembly_cell__srv__MoveToStation_Event__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace drone_assembly_cell

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<drone_assembly_cell::srv::MoveToStation_Event>()
{
  return &::drone_assembly_cell::srv::rosidl_typesupport_cpp::MoveToStation_Event_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, drone_assembly_cell, srv, MoveToStation_Event)() {
  return get_message_type_support_handle<drone_assembly_cell::srv::MoveToStation_Event>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
#include "rosidl_runtime_c/service_type_support_struct.h"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "drone_assembly_cell/srv/detail/move_to_station__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_cpp/service_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace drone_assembly_cell
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _MoveToStation_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _MoveToStation_type_support_ids_t;

static const _MoveToStation_type_support_ids_t _MoveToStation_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _MoveToStation_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _MoveToStation_type_support_symbol_names_t;
#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _MoveToStation_type_support_symbol_names_t _MoveToStation_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, drone_assembly_cell, srv, MoveToStation)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, drone_assembly_cell, srv, MoveToStation)),
  }
};

typedef struct _MoveToStation_type_support_data_t
{
  void * data[2];
} _MoveToStation_type_support_data_t;

static _MoveToStation_type_support_data_t _MoveToStation_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _MoveToStation_service_typesupport_map = {
  2,
  "drone_assembly_cell",
  &_MoveToStation_service_typesupport_ids.typesupport_identifier[0],
  &_MoveToStation_service_typesupport_symbol_names.symbol_name[0],
  &_MoveToStation_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t MoveToStation_service_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_MoveToStation_service_typesupport_map),
  ::rosidl_typesupport_cpp::get_service_typesupport_handle_function,
  ::rosidl_typesupport_cpp::get_message_type_support_handle<drone_assembly_cell::srv::MoveToStation_Request>(),
  ::rosidl_typesupport_cpp::get_message_type_support_handle<drone_assembly_cell::srv::MoveToStation_Response>(),
  ::rosidl_typesupport_cpp::get_message_type_support_handle<drone_assembly_cell::srv::MoveToStation_Event>(),
  &::rosidl_typesupport_cpp::service_create_event_message<drone_assembly_cell::srv::MoveToStation>,
  &::rosidl_typesupport_cpp::service_destroy_event_message<drone_assembly_cell::srv::MoveToStation>,
  &drone_assembly_cell__srv__MoveToStation__get_type_hash,
  &drone_assembly_cell__srv__MoveToStation__get_type_description,
  &drone_assembly_cell__srv__MoveToStation__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace drone_assembly_cell

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<drone_assembly_cell::srv::MoveToStation>()
{
  return &::drone_assembly_cell::srv::rosidl_typesupport_cpp::MoveToStation_service_type_support_handle;
}

}  // namespace rosidl_typesupport_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_cpp, drone_assembly_cell, srv, MoveToStation)() {
  return ::rosidl_typesupport_cpp::get_service_type_support_handle<drone_assembly_cell::srv::MoveToStation>();
}

#ifdef __cplusplus
}
#endif
