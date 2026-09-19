#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



#[link(name = "drone_assembly_cell__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__drone_assembly_cell__srv__MoveToStation_Request() -> *const std::ffi::c_void;
}

#[link(name = "drone_assembly_cell__rosidl_generator_c")]
extern "C" {
    fn drone_assembly_cell__srv__MoveToStation_Request__init(msg: *mut MoveToStation_Request) -> bool;
    fn drone_assembly_cell__srv__MoveToStation_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<MoveToStation_Request>, size: usize) -> bool;
    fn drone_assembly_cell__srv__MoveToStation_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<MoveToStation_Request>);
    fn drone_assembly_cell__srv__MoveToStation_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<MoveToStation_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<MoveToStation_Request>) -> bool;
}

// Corresponds to drone_assembly_cell__srv__MoveToStation_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct MoveToStation_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub drone_id: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub target_station: i32,

}



impl Default for MoveToStation_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !drone_assembly_cell__srv__MoveToStation_Request__init(&mut msg as *mut _) {
        panic!("Call to drone_assembly_cell__srv__MoveToStation_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for MoveToStation_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { drone_assembly_cell__srv__MoveToStation_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { drone_assembly_cell__srv__MoveToStation_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { drone_assembly_cell__srv__MoveToStation_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for MoveToStation_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for MoveToStation_Request where Self: Sized {
  const TYPE_NAME: &'static str = "drone_assembly_cell/srv/MoveToStation_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__drone_assembly_cell__srv__MoveToStation_Request() }
  }
}


#[link(name = "drone_assembly_cell__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__drone_assembly_cell__srv__MoveToStation_Response() -> *const std::ffi::c_void;
}

#[link(name = "drone_assembly_cell__rosidl_generator_c")]
extern "C" {
    fn drone_assembly_cell__srv__MoveToStation_Response__init(msg: *mut MoveToStation_Response) -> bool;
    fn drone_assembly_cell__srv__MoveToStation_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<MoveToStation_Response>, size: usize) -> bool;
    fn drone_assembly_cell__srv__MoveToStation_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<MoveToStation_Response>);
    fn drone_assembly_cell__srv__MoveToStation_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<MoveToStation_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<MoveToStation_Response>) -> bool;
}

// Corresponds to drone_assembly_cell__srv__MoveToStation_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct MoveToStation_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub success: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub message: rosidl_runtime_rs::String,

}



impl Default for MoveToStation_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !drone_assembly_cell__srv__MoveToStation_Response__init(&mut msg as *mut _) {
        panic!("Call to drone_assembly_cell__srv__MoveToStation_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for MoveToStation_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { drone_assembly_cell__srv__MoveToStation_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { drone_assembly_cell__srv__MoveToStation_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { drone_assembly_cell__srv__MoveToStation_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for MoveToStation_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for MoveToStation_Response where Self: Sized {
  const TYPE_NAME: &'static str = "drone_assembly_cell/srv/MoveToStation_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__drone_assembly_cell__srv__MoveToStation_Response() }
  }
}






#[link(name = "drone_assembly_cell__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__drone_assembly_cell__srv__MoveToStation() -> *const std::ffi::c_void;
}

// Corresponds to drone_assembly_cell__srv__MoveToStation
#[allow(missing_docs, non_camel_case_types)]
pub struct MoveToStation;

impl rosidl_runtime_rs::Service for MoveToStation {
    type Request = MoveToStation_Request;
    type Response = MoveToStation_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__drone_assembly_cell__srv__MoveToStation() }
    }
}


