// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from drone_assembly_cell:srv/MoveToStation.idl
// generated code does not contain a copyright notice
#include "drone_assembly_cell/srv/detail/move_to_station__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `drone_id`
#include "rosidl_runtime_c/string_functions.h"

bool
drone_assembly_cell__srv__MoveToStation_Request__init(drone_assembly_cell__srv__MoveToStation_Request * msg)
{
  if (!msg) {
    return false;
  }
  // drone_id
  if (!rosidl_runtime_c__String__init(&msg->drone_id)) {
    drone_assembly_cell__srv__MoveToStation_Request__fini(msg);
    return false;
  }
  // target_station
  return true;
}

void
drone_assembly_cell__srv__MoveToStation_Request__fini(drone_assembly_cell__srv__MoveToStation_Request * msg)
{
  if (!msg) {
    return;
  }
  // drone_id
  rosidl_runtime_c__String__fini(&msg->drone_id);
  // target_station
}

bool
drone_assembly_cell__srv__MoveToStation_Request__are_equal(const drone_assembly_cell__srv__MoveToStation_Request * lhs, const drone_assembly_cell__srv__MoveToStation_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // drone_id
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->drone_id), &(rhs->drone_id)))
  {
    return false;
  }
  // target_station
  if (lhs->target_station != rhs->target_station) {
    return false;
  }
  return true;
}

bool
drone_assembly_cell__srv__MoveToStation_Request__copy(
  const drone_assembly_cell__srv__MoveToStation_Request * input,
  drone_assembly_cell__srv__MoveToStation_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // drone_id
  if (!rosidl_runtime_c__String__copy(
      &(input->drone_id), &(output->drone_id)))
  {
    return false;
  }
  // target_station
  output->target_station = input->target_station;
  return true;
}

drone_assembly_cell__srv__MoveToStation_Request *
drone_assembly_cell__srv__MoveToStation_Request__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  drone_assembly_cell__srv__MoveToStation_Request * msg = (drone_assembly_cell__srv__MoveToStation_Request *)allocator.allocate(sizeof(drone_assembly_cell__srv__MoveToStation_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(drone_assembly_cell__srv__MoveToStation_Request));
  bool success = drone_assembly_cell__srv__MoveToStation_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
drone_assembly_cell__srv__MoveToStation_Request__destroy(drone_assembly_cell__srv__MoveToStation_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    drone_assembly_cell__srv__MoveToStation_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
drone_assembly_cell__srv__MoveToStation_Request__Sequence__init(drone_assembly_cell__srv__MoveToStation_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  drone_assembly_cell__srv__MoveToStation_Request * data = NULL;

  if (size) {
    data = (drone_assembly_cell__srv__MoveToStation_Request *)allocator.zero_allocate(size, sizeof(drone_assembly_cell__srv__MoveToStation_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = drone_assembly_cell__srv__MoveToStation_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        drone_assembly_cell__srv__MoveToStation_Request__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
drone_assembly_cell__srv__MoveToStation_Request__Sequence__fini(drone_assembly_cell__srv__MoveToStation_Request__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      drone_assembly_cell__srv__MoveToStation_Request__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

drone_assembly_cell__srv__MoveToStation_Request__Sequence *
drone_assembly_cell__srv__MoveToStation_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  drone_assembly_cell__srv__MoveToStation_Request__Sequence * array = (drone_assembly_cell__srv__MoveToStation_Request__Sequence *)allocator.allocate(sizeof(drone_assembly_cell__srv__MoveToStation_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = drone_assembly_cell__srv__MoveToStation_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
drone_assembly_cell__srv__MoveToStation_Request__Sequence__destroy(drone_assembly_cell__srv__MoveToStation_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    drone_assembly_cell__srv__MoveToStation_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
drone_assembly_cell__srv__MoveToStation_Request__Sequence__are_equal(const drone_assembly_cell__srv__MoveToStation_Request__Sequence * lhs, const drone_assembly_cell__srv__MoveToStation_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!drone_assembly_cell__srv__MoveToStation_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
drone_assembly_cell__srv__MoveToStation_Request__Sequence__copy(
  const drone_assembly_cell__srv__MoveToStation_Request__Sequence * input,
  drone_assembly_cell__srv__MoveToStation_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(drone_assembly_cell__srv__MoveToStation_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    drone_assembly_cell__srv__MoveToStation_Request * data =
      (drone_assembly_cell__srv__MoveToStation_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!drone_assembly_cell__srv__MoveToStation_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          drone_assembly_cell__srv__MoveToStation_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!drone_assembly_cell__srv__MoveToStation_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `message`
// already included above
// #include "rosidl_runtime_c/string_functions.h"

bool
drone_assembly_cell__srv__MoveToStation_Response__init(drone_assembly_cell__srv__MoveToStation_Response * msg)
{
  if (!msg) {
    return false;
  }
  // success
  // message
  if (!rosidl_runtime_c__String__init(&msg->message)) {
    drone_assembly_cell__srv__MoveToStation_Response__fini(msg);
    return false;
  }
  return true;
}

void
drone_assembly_cell__srv__MoveToStation_Response__fini(drone_assembly_cell__srv__MoveToStation_Response * msg)
{
  if (!msg) {
    return;
  }
  // success
  // message
  rosidl_runtime_c__String__fini(&msg->message);
}

bool
drone_assembly_cell__srv__MoveToStation_Response__are_equal(const drone_assembly_cell__srv__MoveToStation_Response * lhs, const drone_assembly_cell__srv__MoveToStation_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // success
  if (lhs->success != rhs->success) {
    return false;
  }
  // message
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->message), &(rhs->message)))
  {
    return false;
  }
  return true;
}

bool
drone_assembly_cell__srv__MoveToStation_Response__copy(
  const drone_assembly_cell__srv__MoveToStation_Response * input,
  drone_assembly_cell__srv__MoveToStation_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // success
  output->success = input->success;
  // message
  if (!rosidl_runtime_c__String__copy(
      &(input->message), &(output->message)))
  {
    return false;
  }
  return true;
}

drone_assembly_cell__srv__MoveToStation_Response *
drone_assembly_cell__srv__MoveToStation_Response__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  drone_assembly_cell__srv__MoveToStation_Response * msg = (drone_assembly_cell__srv__MoveToStation_Response *)allocator.allocate(sizeof(drone_assembly_cell__srv__MoveToStation_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(drone_assembly_cell__srv__MoveToStation_Response));
  bool success = drone_assembly_cell__srv__MoveToStation_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
drone_assembly_cell__srv__MoveToStation_Response__destroy(drone_assembly_cell__srv__MoveToStation_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    drone_assembly_cell__srv__MoveToStation_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
drone_assembly_cell__srv__MoveToStation_Response__Sequence__init(drone_assembly_cell__srv__MoveToStation_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  drone_assembly_cell__srv__MoveToStation_Response * data = NULL;

  if (size) {
    data = (drone_assembly_cell__srv__MoveToStation_Response *)allocator.zero_allocate(size, sizeof(drone_assembly_cell__srv__MoveToStation_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = drone_assembly_cell__srv__MoveToStation_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        drone_assembly_cell__srv__MoveToStation_Response__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
drone_assembly_cell__srv__MoveToStation_Response__Sequence__fini(drone_assembly_cell__srv__MoveToStation_Response__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      drone_assembly_cell__srv__MoveToStation_Response__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

drone_assembly_cell__srv__MoveToStation_Response__Sequence *
drone_assembly_cell__srv__MoveToStation_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  drone_assembly_cell__srv__MoveToStation_Response__Sequence * array = (drone_assembly_cell__srv__MoveToStation_Response__Sequence *)allocator.allocate(sizeof(drone_assembly_cell__srv__MoveToStation_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = drone_assembly_cell__srv__MoveToStation_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
drone_assembly_cell__srv__MoveToStation_Response__Sequence__destroy(drone_assembly_cell__srv__MoveToStation_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    drone_assembly_cell__srv__MoveToStation_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
drone_assembly_cell__srv__MoveToStation_Response__Sequence__are_equal(const drone_assembly_cell__srv__MoveToStation_Response__Sequence * lhs, const drone_assembly_cell__srv__MoveToStation_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!drone_assembly_cell__srv__MoveToStation_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
drone_assembly_cell__srv__MoveToStation_Response__Sequence__copy(
  const drone_assembly_cell__srv__MoveToStation_Response__Sequence * input,
  drone_assembly_cell__srv__MoveToStation_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(drone_assembly_cell__srv__MoveToStation_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    drone_assembly_cell__srv__MoveToStation_Response * data =
      (drone_assembly_cell__srv__MoveToStation_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!drone_assembly_cell__srv__MoveToStation_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          drone_assembly_cell__srv__MoveToStation_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!drone_assembly_cell__srv__MoveToStation_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `info`
#include "service_msgs/msg/detail/service_event_info__functions.h"
// Member `request`
// Member `response`
// already included above
// #include "drone_assembly_cell/srv/detail/move_to_station__functions.h"

bool
drone_assembly_cell__srv__MoveToStation_Event__init(drone_assembly_cell__srv__MoveToStation_Event * msg)
{
  if (!msg) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__init(&msg->info)) {
    drone_assembly_cell__srv__MoveToStation_Event__fini(msg);
    return false;
  }
  // request
  if (!drone_assembly_cell__srv__MoveToStation_Request__Sequence__init(&msg->request, 0)) {
    drone_assembly_cell__srv__MoveToStation_Event__fini(msg);
    return false;
  }
  // response
  if (!drone_assembly_cell__srv__MoveToStation_Response__Sequence__init(&msg->response, 0)) {
    drone_assembly_cell__srv__MoveToStation_Event__fini(msg);
    return false;
  }
  return true;
}

void
drone_assembly_cell__srv__MoveToStation_Event__fini(drone_assembly_cell__srv__MoveToStation_Event * msg)
{
  if (!msg) {
    return;
  }
  // info
  service_msgs__msg__ServiceEventInfo__fini(&msg->info);
  // request
  drone_assembly_cell__srv__MoveToStation_Request__Sequence__fini(&msg->request);
  // response
  drone_assembly_cell__srv__MoveToStation_Response__Sequence__fini(&msg->response);
}

bool
drone_assembly_cell__srv__MoveToStation_Event__are_equal(const drone_assembly_cell__srv__MoveToStation_Event * lhs, const drone_assembly_cell__srv__MoveToStation_Event * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__are_equal(
      &(lhs->info), &(rhs->info)))
  {
    return false;
  }
  // request
  if (!drone_assembly_cell__srv__MoveToStation_Request__Sequence__are_equal(
      &(lhs->request), &(rhs->request)))
  {
    return false;
  }
  // response
  if (!drone_assembly_cell__srv__MoveToStation_Response__Sequence__are_equal(
      &(lhs->response), &(rhs->response)))
  {
    return false;
  }
  return true;
}

bool
drone_assembly_cell__srv__MoveToStation_Event__copy(
  const drone_assembly_cell__srv__MoveToStation_Event * input,
  drone_assembly_cell__srv__MoveToStation_Event * output)
{
  if (!input || !output) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__copy(
      &(input->info), &(output->info)))
  {
    return false;
  }
  // request
  if (!drone_assembly_cell__srv__MoveToStation_Request__Sequence__copy(
      &(input->request), &(output->request)))
  {
    return false;
  }
  // response
  if (!drone_assembly_cell__srv__MoveToStation_Response__Sequence__copy(
      &(input->response), &(output->response)))
  {
    return false;
  }
  return true;
}

drone_assembly_cell__srv__MoveToStation_Event *
drone_assembly_cell__srv__MoveToStation_Event__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  drone_assembly_cell__srv__MoveToStation_Event * msg = (drone_assembly_cell__srv__MoveToStation_Event *)allocator.allocate(sizeof(drone_assembly_cell__srv__MoveToStation_Event), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(drone_assembly_cell__srv__MoveToStation_Event));
  bool success = drone_assembly_cell__srv__MoveToStation_Event__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
drone_assembly_cell__srv__MoveToStation_Event__destroy(drone_assembly_cell__srv__MoveToStation_Event * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    drone_assembly_cell__srv__MoveToStation_Event__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
drone_assembly_cell__srv__MoveToStation_Event__Sequence__init(drone_assembly_cell__srv__MoveToStation_Event__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  drone_assembly_cell__srv__MoveToStation_Event * data = NULL;

  if (size) {
    data = (drone_assembly_cell__srv__MoveToStation_Event *)allocator.zero_allocate(size, sizeof(drone_assembly_cell__srv__MoveToStation_Event), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = drone_assembly_cell__srv__MoveToStation_Event__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        drone_assembly_cell__srv__MoveToStation_Event__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
drone_assembly_cell__srv__MoveToStation_Event__Sequence__fini(drone_assembly_cell__srv__MoveToStation_Event__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      drone_assembly_cell__srv__MoveToStation_Event__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

drone_assembly_cell__srv__MoveToStation_Event__Sequence *
drone_assembly_cell__srv__MoveToStation_Event__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  drone_assembly_cell__srv__MoveToStation_Event__Sequence * array = (drone_assembly_cell__srv__MoveToStation_Event__Sequence *)allocator.allocate(sizeof(drone_assembly_cell__srv__MoveToStation_Event__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = drone_assembly_cell__srv__MoveToStation_Event__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
drone_assembly_cell__srv__MoveToStation_Event__Sequence__destroy(drone_assembly_cell__srv__MoveToStation_Event__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    drone_assembly_cell__srv__MoveToStation_Event__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
drone_assembly_cell__srv__MoveToStation_Event__Sequence__are_equal(const drone_assembly_cell__srv__MoveToStation_Event__Sequence * lhs, const drone_assembly_cell__srv__MoveToStation_Event__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!drone_assembly_cell__srv__MoveToStation_Event__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
drone_assembly_cell__srv__MoveToStation_Event__Sequence__copy(
  const drone_assembly_cell__srv__MoveToStation_Event__Sequence * input,
  drone_assembly_cell__srv__MoveToStation_Event__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(drone_assembly_cell__srv__MoveToStation_Event);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    drone_assembly_cell__srv__MoveToStation_Event * data =
      (drone_assembly_cell__srv__MoveToStation_Event *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!drone_assembly_cell__srv__MoveToStation_Event__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          drone_assembly_cell__srv__MoveToStation_Event__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!drone_assembly_cell__srv__MoveToStation_Event__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
