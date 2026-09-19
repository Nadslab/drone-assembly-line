#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};




// Corresponds to drone_assembly_cell__srv__MoveToStation_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct MoveToStation_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub drone_id: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub target_station: i32,

}



impl Default for MoveToStation_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::MoveToStation_Request::default())
  }
}

impl rosidl_runtime_rs::Message for MoveToStation_Request {
  type RmwMsg = super::srv::rmw::MoveToStation_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        drone_id: msg.drone_id.as_str().into(),
        target_station: msg.target_station,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        drone_id: msg.drone_id.as_str().into(),
      target_station: msg.target_station,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      drone_id: msg.drone_id.to_string(),
      target_station: msg.target_station,
    }
  }
}


// Corresponds to drone_assembly_cell__srv__MoveToStation_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct MoveToStation_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub success: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub message: std::string::String,

}



impl Default for MoveToStation_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::MoveToStation_Response::default())
  }
}

impl rosidl_runtime_rs::Message for MoveToStation_Response {
  type RmwMsg = super::srv::rmw::MoveToStation_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        success: msg.success,
        message: msg.message.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      success: msg.success,
        message: msg.message.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      success: msg.success,
      message: msg.message.to_string(),
    }
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


