
#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};


#[link(name = "drone_assembly_cell__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__drone_assembly_cell__action__PickPart_Goal() -> *const std::ffi::c_void;
}

#[link(name = "drone_assembly_cell__rosidl_generator_c")]
extern "C" {
    fn drone_assembly_cell__action__PickPart_Goal__init(msg: *mut PickPart_Goal) -> bool;
    fn drone_assembly_cell__action__PickPart_Goal__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<PickPart_Goal>, size: usize) -> bool;
    fn drone_assembly_cell__action__PickPart_Goal__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<PickPart_Goal>);
    fn drone_assembly_cell__action__PickPart_Goal__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<PickPart_Goal>, out_seq: *mut rosidl_runtime_rs::Sequence<PickPart_Goal>) -> bool;
}

// Corresponds to drone_assembly_cell__action__PickPart_Goal
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct PickPart_Goal {

    // This member is not documented.
    #[allow(missing_docs)]
    pub source_pose: geometry_msgs::msg::rmw::PoseStamped,


    // This member is not documented.
    #[allow(missing_docs)]
    pub approach_height: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub gripper_close_pos: f64,

}



impl Default for PickPart_Goal {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !drone_assembly_cell__action__PickPart_Goal__init(&mut msg as *mut _) {
        panic!("Call to drone_assembly_cell__action__PickPart_Goal__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for PickPart_Goal {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { drone_assembly_cell__action__PickPart_Goal__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { drone_assembly_cell__action__PickPart_Goal__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { drone_assembly_cell__action__PickPart_Goal__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for PickPart_Goal {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for PickPart_Goal where Self: Sized {
  const TYPE_NAME: &'static str = "drone_assembly_cell/action/PickPart_Goal";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__drone_assembly_cell__action__PickPart_Goal() }
  }
}


#[link(name = "drone_assembly_cell__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__drone_assembly_cell__action__PickPart_Result() -> *const std::ffi::c_void;
}

#[link(name = "drone_assembly_cell__rosidl_generator_c")]
extern "C" {
    fn drone_assembly_cell__action__PickPart_Result__init(msg: *mut PickPart_Result) -> bool;
    fn drone_assembly_cell__action__PickPart_Result__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<PickPart_Result>, size: usize) -> bool;
    fn drone_assembly_cell__action__PickPart_Result__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<PickPart_Result>);
    fn drone_assembly_cell__action__PickPart_Result__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<PickPart_Result>, out_seq: *mut rosidl_runtime_rs::Sequence<PickPart_Result>) -> bool;
}

// Corresponds to drone_assembly_cell__action__PickPart_Result
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct PickPart_Result {

    // This member is not documented.
    #[allow(missing_docs)]
    pub success: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub message: rosidl_runtime_rs::String,

}



impl Default for PickPart_Result {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !drone_assembly_cell__action__PickPart_Result__init(&mut msg as *mut _) {
        panic!("Call to drone_assembly_cell__action__PickPart_Result__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for PickPart_Result {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { drone_assembly_cell__action__PickPart_Result__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { drone_assembly_cell__action__PickPart_Result__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { drone_assembly_cell__action__PickPart_Result__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for PickPart_Result {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for PickPart_Result where Self: Sized {
  const TYPE_NAME: &'static str = "drone_assembly_cell/action/PickPart_Result";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__drone_assembly_cell__action__PickPart_Result() }
  }
}


#[link(name = "drone_assembly_cell__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__drone_assembly_cell__action__PickPart_Feedback() -> *const std::ffi::c_void;
}

#[link(name = "drone_assembly_cell__rosidl_generator_c")]
extern "C" {
    fn drone_assembly_cell__action__PickPart_Feedback__init(msg: *mut PickPart_Feedback) -> bool;
    fn drone_assembly_cell__action__PickPart_Feedback__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<PickPart_Feedback>, size: usize) -> bool;
    fn drone_assembly_cell__action__PickPart_Feedback__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<PickPart_Feedback>);
    fn drone_assembly_cell__action__PickPart_Feedback__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<PickPart_Feedback>, out_seq: *mut rosidl_runtime_rs::Sequence<PickPart_Feedback>) -> bool;
}

// Corresponds to drone_assembly_cell__action__PickPart_Feedback
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct PickPart_Feedback {

    // This member is not documented.
    #[allow(missing_docs)]
    pub phase: rosidl_runtime_rs::String,

}



impl Default for PickPart_Feedback {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !drone_assembly_cell__action__PickPart_Feedback__init(&mut msg as *mut _) {
        panic!("Call to drone_assembly_cell__action__PickPart_Feedback__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for PickPart_Feedback {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { drone_assembly_cell__action__PickPart_Feedback__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { drone_assembly_cell__action__PickPart_Feedback__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { drone_assembly_cell__action__PickPart_Feedback__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for PickPart_Feedback {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for PickPart_Feedback where Self: Sized {
  const TYPE_NAME: &'static str = "drone_assembly_cell/action/PickPart_Feedback";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__drone_assembly_cell__action__PickPart_Feedback() }
  }
}


#[link(name = "drone_assembly_cell__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__drone_assembly_cell__action__PickPart_FeedbackMessage() -> *const std::ffi::c_void;
}

#[link(name = "drone_assembly_cell__rosidl_generator_c")]
extern "C" {
    fn drone_assembly_cell__action__PickPart_FeedbackMessage__init(msg: *mut PickPart_FeedbackMessage) -> bool;
    fn drone_assembly_cell__action__PickPart_FeedbackMessage__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<PickPart_FeedbackMessage>, size: usize) -> bool;
    fn drone_assembly_cell__action__PickPart_FeedbackMessage__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<PickPart_FeedbackMessage>);
    fn drone_assembly_cell__action__PickPart_FeedbackMessage__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<PickPart_FeedbackMessage>, out_seq: *mut rosidl_runtime_rs::Sequence<PickPart_FeedbackMessage>) -> bool;
}

// Corresponds to drone_assembly_cell__action__PickPart_FeedbackMessage
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct PickPart_FeedbackMessage {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,


    // This member is not documented.
    #[allow(missing_docs)]
    pub feedback: super::super::action::rmw::PickPart_Feedback,

}



impl Default for PickPart_FeedbackMessage {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !drone_assembly_cell__action__PickPart_FeedbackMessage__init(&mut msg as *mut _) {
        panic!("Call to drone_assembly_cell__action__PickPart_FeedbackMessage__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for PickPart_FeedbackMessage {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { drone_assembly_cell__action__PickPart_FeedbackMessage__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { drone_assembly_cell__action__PickPart_FeedbackMessage__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { drone_assembly_cell__action__PickPart_FeedbackMessage__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for PickPart_FeedbackMessage {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for PickPart_FeedbackMessage where Self: Sized {
  const TYPE_NAME: &'static str = "drone_assembly_cell/action/PickPart_FeedbackMessage";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__drone_assembly_cell__action__PickPart_FeedbackMessage() }
  }
}


#[link(name = "drone_assembly_cell__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__drone_assembly_cell__action__PlacePart_Goal() -> *const std::ffi::c_void;
}

#[link(name = "drone_assembly_cell__rosidl_generator_c")]
extern "C" {
    fn drone_assembly_cell__action__PlacePart_Goal__init(msg: *mut PlacePart_Goal) -> bool;
    fn drone_assembly_cell__action__PlacePart_Goal__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<PlacePart_Goal>, size: usize) -> bool;
    fn drone_assembly_cell__action__PlacePart_Goal__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<PlacePart_Goal>);
    fn drone_assembly_cell__action__PlacePart_Goal__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<PlacePart_Goal>, out_seq: *mut rosidl_runtime_rs::Sequence<PlacePart_Goal>) -> bool;
}

// Corresponds to drone_assembly_cell__action__PlacePart_Goal
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct PlacePart_Goal {

    // This member is not documented.
    #[allow(missing_docs)]
    pub target_pose: geometry_msgs::msg::rmw::PoseStamped,


    // This member is not documented.
    #[allow(missing_docs)]
    pub approach_height: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub gripper_open_pos: f64,

}



impl Default for PlacePart_Goal {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !drone_assembly_cell__action__PlacePart_Goal__init(&mut msg as *mut _) {
        panic!("Call to drone_assembly_cell__action__PlacePart_Goal__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for PlacePart_Goal {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { drone_assembly_cell__action__PlacePart_Goal__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { drone_assembly_cell__action__PlacePart_Goal__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { drone_assembly_cell__action__PlacePart_Goal__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for PlacePart_Goal {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for PlacePart_Goal where Self: Sized {
  const TYPE_NAME: &'static str = "drone_assembly_cell/action/PlacePart_Goal";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__drone_assembly_cell__action__PlacePart_Goal() }
  }
}


#[link(name = "drone_assembly_cell__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__drone_assembly_cell__action__PlacePart_Result() -> *const std::ffi::c_void;
}

#[link(name = "drone_assembly_cell__rosidl_generator_c")]
extern "C" {
    fn drone_assembly_cell__action__PlacePart_Result__init(msg: *mut PlacePart_Result) -> bool;
    fn drone_assembly_cell__action__PlacePart_Result__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<PlacePart_Result>, size: usize) -> bool;
    fn drone_assembly_cell__action__PlacePart_Result__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<PlacePart_Result>);
    fn drone_assembly_cell__action__PlacePart_Result__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<PlacePart_Result>, out_seq: *mut rosidl_runtime_rs::Sequence<PlacePart_Result>) -> bool;
}

// Corresponds to drone_assembly_cell__action__PlacePart_Result
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct PlacePart_Result {

    // This member is not documented.
    #[allow(missing_docs)]
    pub success: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub message: rosidl_runtime_rs::String,

}



impl Default for PlacePart_Result {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !drone_assembly_cell__action__PlacePart_Result__init(&mut msg as *mut _) {
        panic!("Call to drone_assembly_cell__action__PlacePart_Result__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for PlacePart_Result {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { drone_assembly_cell__action__PlacePart_Result__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { drone_assembly_cell__action__PlacePart_Result__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { drone_assembly_cell__action__PlacePart_Result__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for PlacePart_Result {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for PlacePart_Result where Self: Sized {
  const TYPE_NAME: &'static str = "drone_assembly_cell/action/PlacePart_Result";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__drone_assembly_cell__action__PlacePart_Result() }
  }
}


#[link(name = "drone_assembly_cell__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__drone_assembly_cell__action__PlacePart_Feedback() -> *const std::ffi::c_void;
}

#[link(name = "drone_assembly_cell__rosidl_generator_c")]
extern "C" {
    fn drone_assembly_cell__action__PlacePart_Feedback__init(msg: *mut PlacePart_Feedback) -> bool;
    fn drone_assembly_cell__action__PlacePart_Feedback__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<PlacePart_Feedback>, size: usize) -> bool;
    fn drone_assembly_cell__action__PlacePart_Feedback__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<PlacePart_Feedback>);
    fn drone_assembly_cell__action__PlacePart_Feedback__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<PlacePart_Feedback>, out_seq: *mut rosidl_runtime_rs::Sequence<PlacePart_Feedback>) -> bool;
}

// Corresponds to drone_assembly_cell__action__PlacePart_Feedback
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct PlacePart_Feedback {

    // This member is not documented.
    #[allow(missing_docs)]
    pub phase: rosidl_runtime_rs::String,

}



impl Default for PlacePart_Feedback {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !drone_assembly_cell__action__PlacePart_Feedback__init(&mut msg as *mut _) {
        panic!("Call to drone_assembly_cell__action__PlacePart_Feedback__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for PlacePart_Feedback {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { drone_assembly_cell__action__PlacePart_Feedback__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { drone_assembly_cell__action__PlacePart_Feedback__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { drone_assembly_cell__action__PlacePart_Feedback__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for PlacePart_Feedback {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for PlacePart_Feedback where Self: Sized {
  const TYPE_NAME: &'static str = "drone_assembly_cell/action/PlacePart_Feedback";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__drone_assembly_cell__action__PlacePart_Feedback() }
  }
}


#[link(name = "drone_assembly_cell__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__drone_assembly_cell__action__PlacePart_FeedbackMessage() -> *const std::ffi::c_void;
}

#[link(name = "drone_assembly_cell__rosidl_generator_c")]
extern "C" {
    fn drone_assembly_cell__action__PlacePart_FeedbackMessage__init(msg: *mut PlacePart_FeedbackMessage) -> bool;
    fn drone_assembly_cell__action__PlacePart_FeedbackMessage__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<PlacePart_FeedbackMessage>, size: usize) -> bool;
    fn drone_assembly_cell__action__PlacePart_FeedbackMessage__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<PlacePart_FeedbackMessage>);
    fn drone_assembly_cell__action__PlacePart_FeedbackMessage__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<PlacePart_FeedbackMessage>, out_seq: *mut rosidl_runtime_rs::Sequence<PlacePart_FeedbackMessage>) -> bool;
}

// Corresponds to drone_assembly_cell__action__PlacePart_FeedbackMessage
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct PlacePart_FeedbackMessage {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,


    // This member is not documented.
    #[allow(missing_docs)]
    pub feedback: super::super::action::rmw::PlacePart_Feedback,

}



impl Default for PlacePart_FeedbackMessage {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !drone_assembly_cell__action__PlacePart_FeedbackMessage__init(&mut msg as *mut _) {
        panic!("Call to drone_assembly_cell__action__PlacePart_FeedbackMessage__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for PlacePart_FeedbackMessage {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { drone_assembly_cell__action__PlacePart_FeedbackMessage__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { drone_assembly_cell__action__PlacePart_FeedbackMessage__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { drone_assembly_cell__action__PlacePart_FeedbackMessage__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for PlacePart_FeedbackMessage {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for PlacePart_FeedbackMessage where Self: Sized {
  const TYPE_NAME: &'static str = "drone_assembly_cell/action/PlacePart_FeedbackMessage";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__drone_assembly_cell__action__PlacePart_FeedbackMessage() }
  }
}




#[link(name = "drone_assembly_cell__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__drone_assembly_cell__action__PickPart_SendGoal_Request() -> *const std::ffi::c_void;
}

#[link(name = "drone_assembly_cell__rosidl_generator_c")]
extern "C" {
    fn drone_assembly_cell__action__PickPart_SendGoal_Request__init(msg: *mut PickPart_SendGoal_Request) -> bool;
    fn drone_assembly_cell__action__PickPart_SendGoal_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<PickPart_SendGoal_Request>, size: usize) -> bool;
    fn drone_assembly_cell__action__PickPart_SendGoal_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<PickPart_SendGoal_Request>);
    fn drone_assembly_cell__action__PickPart_SendGoal_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<PickPart_SendGoal_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<PickPart_SendGoal_Request>) -> bool;
}

// Corresponds to drone_assembly_cell__action__PickPart_SendGoal_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct PickPart_SendGoal_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,


    // This member is not documented.
    #[allow(missing_docs)]
    pub goal: super::super::action::rmw::PickPart_Goal,

}



impl Default for PickPart_SendGoal_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !drone_assembly_cell__action__PickPart_SendGoal_Request__init(&mut msg as *mut _) {
        panic!("Call to drone_assembly_cell__action__PickPart_SendGoal_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for PickPart_SendGoal_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { drone_assembly_cell__action__PickPart_SendGoal_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { drone_assembly_cell__action__PickPart_SendGoal_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { drone_assembly_cell__action__PickPart_SendGoal_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for PickPart_SendGoal_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for PickPart_SendGoal_Request where Self: Sized {
  const TYPE_NAME: &'static str = "drone_assembly_cell/action/PickPart_SendGoal_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__drone_assembly_cell__action__PickPart_SendGoal_Request() }
  }
}


#[link(name = "drone_assembly_cell__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__drone_assembly_cell__action__PickPart_SendGoal_Response() -> *const std::ffi::c_void;
}

#[link(name = "drone_assembly_cell__rosidl_generator_c")]
extern "C" {
    fn drone_assembly_cell__action__PickPart_SendGoal_Response__init(msg: *mut PickPart_SendGoal_Response) -> bool;
    fn drone_assembly_cell__action__PickPart_SendGoal_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<PickPart_SendGoal_Response>, size: usize) -> bool;
    fn drone_assembly_cell__action__PickPart_SendGoal_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<PickPart_SendGoal_Response>);
    fn drone_assembly_cell__action__PickPart_SendGoal_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<PickPart_SendGoal_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<PickPart_SendGoal_Response>) -> bool;
}

// Corresponds to drone_assembly_cell__action__PickPart_SendGoal_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct PickPart_SendGoal_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub accepted: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::rmw::Time,

}



impl Default for PickPart_SendGoal_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !drone_assembly_cell__action__PickPart_SendGoal_Response__init(&mut msg as *mut _) {
        panic!("Call to drone_assembly_cell__action__PickPart_SendGoal_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for PickPart_SendGoal_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { drone_assembly_cell__action__PickPart_SendGoal_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { drone_assembly_cell__action__PickPart_SendGoal_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { drone_assembly_cell__action__PickPart_SendGoal_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for PickPart_SendGoal_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for PickPart_SendGoal_Response where Self: Sized {
  const TYPE_NAME: &'static str = "drone_assembly_cell/action/PickPart_SendGoal_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__drone_assembly_cell__action__PickPart_SendGoal_Response() }
  }
}


#[link(name = "drone_assembly_cell__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__drone_assembly_cell__action__PickPart_GetResult_Request() -> *const std::ffi::c_void;
}

#[link(name = "drone_assembly_cell__rosidl_generator_c")]
extern "C" {
    fn drone_assembly_cell__action__PickPart_GetResult_Request__init(msg: *mut PickPart_GetResult_Request) -> bool;
    fn drone_assembly_cell__action__PickPart_GetResult_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<PickPart_GetResult_Request>, size: usize) -> bool;
    fn drone_assembly_cell__action__PickPart_GetResult_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<PickPart_GetResult_Request>);
    fn drone_assembly_cell__action__PickPart_GetResult_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<PickPart_GetResult_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<PickPart_GetResult_Request>) -> bool;
}

// Corresponds to drone_assembly_cell__action__PickPart_GetResult_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct PickPart_GetResult_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,

}



impl Default for PickPart_GetResult_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !drone_assembly_cell__action__PickPart_GetResult_Request__init(&mut msg as *mut _) {
        panic!("Call to drone_assembly_cell__action__PickPart_GetResult_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for PickPart_GetResult_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { drone_assembly_cell__action__PickPart_GetResult_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { drone_assembly_cell__action__PickPart_GetResult_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { drone_assembly_cell__action__PickPart_GetResult_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for PickPart_GetResult_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for PickPart_GetResult_Request where Self: Sized {
  const TYPE_NAME: &'static str = "drone_assembly_cell/action/PickPart_GetResult_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__drone_assembly_cell__action__PickPart_GetResult_Request() }
  }
}


#[link(name = "drone_assembly_cell__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__drone_assembly_cell__action__PickPart_GetResult_Response() -> *const std::ffi::c_void;
}

#[link(name = "drone_assembly_cell__rosidl_generator_c")]
extern "C" {
    fn drone_assembly_cell__action__PickPart_GetResult_Response__init(msg: *mut PickPart_GetResult_Response) -> bool;
    fn drone_assembly_cell__action__PickPart_GetResult_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<PickPart_GetResult_Response>, size: usize) -> bool;
    fn drone_assembly_cell__action__PickPart_GetResult_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<PickPart_GetResult_Response>);
    fn drone_assembly_cell__action__PickPart_GetResult_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<PickPart_GetResult_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<PickPart_GetResult_Response>) -> bool;
}

// Corresponds to drone_assembly_cell__action__PickPart_GetResult_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct PickPart_GetResult_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub status: i8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub result: super::super::action::rmw::PickPart_Result,

}



impl Default for PickPart_GetResult_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !drone_assembly_cell__action__PickPart_GetResult_Response__init(&mut msg as *mut _) {
        panic!("Call to drone_assembly_cell__action__PickPart_GetResult_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for PickPart_GetResult_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { drone_assembly_cell__action__PickPart_GetResult_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { drone_assembly_cell__action__PickPart_GetResult_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { drone_assembly_cell__action__PickPart_GetResult_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for PickPart_GetResult_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for PickPart_GetResult_Response where Self: Sized {
  const TYPE_NAME: &'static str = "drone_assembly_cell/action/PickPart_GetResult_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__drone_assembly_cell__action__PickPart_GetResult_Response() }
  }
}


#[link(name = "drone_assembly_cell__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__drone_assembly_cell__action__PlacePart_SendGoal_Request() -> *const std::ffi::c_void;
}

#[link(name = "drone_assembly_cell__rosidl_generator_c")]
extern "C" {
    fn drone_assembly_cell__action__PlacePart_SendGoal_Request__init(msg: *mut PlacePart_SendGoal_Request) -> bool;
    fn drone_assembly_cell__action__PlacePart_SendGoal_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<PlacePart_SendGoal_Request>, size: usize) -> bool;
    fn drone_assembly_cell__action__PlacePart_SendGoal_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<PlacePart_SendGoal_Request>);
    fn drone_assembly_cell__action__PlacePart_SendGoal_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<PlacePart_SendGoal_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<PlacePart_SendGoal_Request>) -> bool;
}

// Corresponds to drone_assembly_cell__action__PlacePart_SendGoal_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct PlacePart_SendGoal_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,


    // This member is not documented.
    #[allow(missing_docs)]
    pub goal: super::super::action::rmw::PlacePart_Goal,

}



impl Default for PlacePart_SendGoal_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !drone_assembly_cell__action__PlacePart_SendGoal_Request__init(&mut msg as *mut _) {
        panic!("Call to drone_assembly_cell__action__PlacePart_SendGoal_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for PlacePart_SendGoal_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { drone_assembly_cell__action__PlacePart_SendGoal_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { drone_assembly_cell__action__PlacePart_SendGoal_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { drone_assembly_cell__action__PlacePart_SendGoal_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for PlacePart_SendGoal_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for PlacePart_SendGoal_Request where Self: Sized {
  const TYPE_NAME: &'static str = "drone_assembly_cell/action/PlacePart_SendGoal_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__drone_assembly_cell__action__PlacePart_SendGoal_Request() }
  }
}


#[link(name = "drone_assembly_cell__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__drone_assembly_cell__action__PlacePart_SendGoal_Response() -> *const std::ffi::c_void;
}

#[link(name = "drone_assembly_cell__rosidl_generator_c")]
extern "C" {
    fn drone_assembly_cell__action__PlacePart_SendGoal_Response__init(msg: *mut PlacePart_SendGoal_Response) -> bool;
    fn drone_assembly_cell__action__PlacePart_SendGoal_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<PlacePart_SendGoal_Response>, size: usize) -> bool;
    fn drone_assembly_cell__action__PlacePart_SendGoal_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<PlacePart_SendGoal_Response>);
    fn drone_assembly_cell__action__PlacePart_SendGoal_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<PlacePart_SendGoal_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<PlacePart_SendGoal_Response>) -> bool;
}

// Corresponds to drone_assembly_cell__action__PlacePart_SendGoal_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct PlacePart_SendGoal_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub accepted: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::rmw::Time,

}



impl Default for PlacePart_SendGoal_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !drone_assembly_cell__action__PlacePart_SendGoal_Response__init(&mut msg as *mut _) {
        panic!("Call to drone_assembly_cell__action__PlacePart_SendGoal_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for PlacePart_SendGoal_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { drone_assembly_cell__action__PlacePart_SendGoal_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { drone_assembly_cell__action__PlacePart_SendGoal_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { drone_assembly_cell__action__PlacePart_SendGoal_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for PlacePart_SendGoal_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for PlacePart_SendGoal_Response where Self: Sized {
  const TYPE_NAME: &'static str = "drone_assembly_cell/action/PlacePart_SendGoal_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__drone_assembly_cell__action__PlacePart_SendGoal_Response() }
  }
}


#[link(name = "drone_assembly_cell__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__drone_assembly_cell__action__PlacePart_GetResult_Request() -> *const std::ffi::c_void;
}

#[link(name = "drone_assembly_cell__rosidl_generator_c")]
extern "C" {
    fn drone_assembly_cell__action__PlacePart_GetResult_Request__init(msg: *mut PlacePart_GetResult_Request) -> bool;
    fn drone_assembly_cell__action__PlacePart_GetResult_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<PlacePart_GetResult_Request>, size: usize) -> bool;
    fn drone_assembly_cell__action__PlacePart_GetResult_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<PlacePart_GetResult_Request>);
    fn drone_assembly_cell__action__PlacePart_GetResult_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<PlacePart_GetResult_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<PlacePart_GetResult_Request>) -> bool;
}

// Corresponds to drone_assembly_cell__action__PlacePart_GetResult_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct PlacePart_GetResult_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,

}



impl Default for PlacePart_GetResult_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !drone_assembly_cell__action__PlacePart_GetResult_Request__init(&mut msg as *mut _) {
        panic!("Call to drone_assembly_cell__action__PlacePart_GetResult_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for PlacePart_GetResult_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { drone_assembly_cell__action__PlacePart_GetResult_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { drone_assembly_cell__action__PlacePart_GetResult_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { drone_assembly_cell__action__PlacePart_GetResult_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for PlacePart_GetResult_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for PlacePart_GetResult_Request where Self: Sized {
  const TYPE_NAME: &'static str = "drone_assembly_cell/action/PlacePart_GetResult_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__drone_assembly_cell__action__PlacePart_GetResult_Request() }
  }
}


#[link(name = "drone_assembly_cell__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__drone_assembly_cell__action__PlacePart_GetResult_Response() -> *const std::ffi::c_void;
}

#[link(name = "drone_assembly_cell__rosidl_generator_c")]
extern "C" {
    fn drone_assembly_cell__action__PlacePart_GetResult_Response__init(msg: *mut PlacePart_GetResult_Response) -> bool;
    fn drone_assembly_cell__action__PlacePart_GetResult_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<PlacePart_GetResult_Response>, size: usize) -> bool;
    fn drone_assembly_cell__action__PlacePart_GetResult_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<PlacePart_GetResult_Response>);
    fn drone_assembly_cell__action__PlacePart_GetResult_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<PlacePart_GetResult_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<PlacePart_GetResult_Response>) -> bool;
}

// Corresponds to drone_assembly_cell__action__PlacePart_GetResult_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct PlacePart_GetResult_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub status: i8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub result: super::super::action::rmw::PlacePart_Result,

}



impl Default for PlacePart_GetResult_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !drone_assembly_cell__action__PlacePart_GetResult_Response__init(&mut msg as *mut _) {
        panic!("Call to drone_assembly_cell__action__PlacePart_GetResult_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for PlacePart_GetResult_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { drone_assembly_cell__action__PlacePart_GetResult_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { drone_assembly_cell__action__PlacePart_GetResult_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { drone_assembly_cell__action__PlacePart_GetResult_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for PlacePart_GetResult_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for PlacePart_GetResult_Response where Self: Sized {
  const TYPE_NAME: &'static str = "drone_assembly_cell/action/PlacePart_GetResult_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__drone_assembly_cell__action__PlacePart_GetResult_Response() }
  }
}






#[link(name = "drone_assembly_cell__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__drone_assembly_cell__action__PickPart_SendGoal() -> *const std::ffi::c_void;
}

// Corresponds to drone_assembly_cell__action__PickPart_SendGoal
#[allow(missing_docs, non_camel_case_types)]
pub struct PickPart_SendGoal;

impl rosidl_runtime_rs::Service for PickPart_SendGoal {
    type Request = PickPart_SendGoal_Request;
    type Response = PickPart_SendGoal_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__drone_assembly_cell__action__PickPart_SendGoal() }
    }
}




#[link(name = "drone_assembly_cell__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__drone_assembly_cell__action__PickPart_GetResult() -> *const std::ffi::c_void;
}

// Corresponds to drone_assembly_cell__action__PickPart_GetResult
#[allow(missing_docs, non_camel_case_types)]
pub struct PickPart_GetResult;

impl rosidl_runtime_rs::Service for PickPart_GetResult {
    type Request = PickPart_GetResult_Request;
    type Response = PickPart_GetResult_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__drone_assembly_cell__action__PickPart_GetResult() }
    }
}




#[link(name = "drone_assembly_cell__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__drone_assembly_cell__action__PlacePart_SendGoal() -> *const std::ffi::c_void;
}

// Corresponds to drone_assembly_cell__action__PlacePart_SendGoal
#[allow(missing_docs, non_camel_case_types)]
pub struct PlacePart_SendGoal;

impl rosidl_runtime_rs::Service for PlacePart_SendGoal {
    type Request = PlacePart_SendGoal_Request;
    type Response = PlacePart_SendGoal_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__drone_assembly_cell__action__PlacePart_SendGoal() }
    }
}




#[link(name = "drone_assembly_cell__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__drone_assembly_cell__action__PlacePart_GetResult() -> *const std::ffi::c_void;
}

// Corresponds to drone_assembly_cell__action__PlacePart_GetResult
#[allow(missing_docs, non_camel_case_types)]
pub struct PlacePart_GetResult;

impl rosidl_runtime_rs::Service for PlacePart_GetResult {
    type Request = PlacePart_GetResult_Request;
    type Response = PlacePart_GetResult_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__drone_assembly_cell__action__PlacePart_GetResult() }
    }
}


