#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



#[link(name = "isaac_ros2_messages__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__isaac_ros2_messages__srv__IsaacPose_Request() -> *const std::ffi::c_void;
}

#[link(name = "isaac_ros2_messages__rosidl_generator_c")]
extern "C" {
    fn isaac_ros2_messages__srv__IsaacPose_Request__init(msg: *mut IsaacPose_Request) -> bool;
    fn isaac_ros2_messages__srv__IsaacPose_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<IsaacPose_Request>, size: usize) -> bool;
    fn isaac_ros2_messages__srv__IsaacPose_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<IsaacPose_Request>);
    fn isaac_ros2_messages__srv__IsaacPose_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<IsaacPose_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<IsaacPose_Request>) -> bool;
}

// Corresponds to isaac_ros2_messages__srv__IsaacPose_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct IsaacPose_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::rmw::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub names: rosidl_runtime_rs::Sequence<rosidl_runtime_rs::String>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub poses: rosidl_runtime_rs::Sequence<geometry_msgs::msg::rmw::Pose>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub velocities: rosidl_runtime_rs::Sequence<geometry_msgs::msg::rmw::Twist>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub scales: rosidl_runtime_rs::Sequence<geometry_msgs::msg::rmw::Vector3>,

}



impl Default for IsaacPose_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !isaac_ros2_messages__srv__IsaacPose_Request__init(&mut msg as *mut _) {
        panic!("Call to isaac_ros2_messages__srv__IsaacPose_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for IsaacPose_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { isaac_ros2_messages__srv__IsaacPose_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { isaac_ros2_messages__srv__IsaacPose_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { isaac_ros2_messages__srv__IsaacPose_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for IsaacPose_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for IsaacPose_Request where Self: Sized {
  const TYPE_NAME: &'static str = "isaac_ros2_messages/srv/IsaacPose_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__isaac_ros2_messages__srv__IsaacPose_Request() }
  }
}


#[link(name = "isaac_ros2_messages__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__isaac_ros2_messages__srv__IsaacPose_Response() -> *const std::ffi::c_void;
}

#[link(name = "isaac_ros2_messages__rosidl_generator_c")]
extern "C" {
    fn isaac_ros2_messages__srv__IsaacPose_Response__init(msg: *mut IsaacPose_Response) -> bool;
    fn isaac_ros2_messages__srv__IsaacPose_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<IsaacPose_Response>, size: usize) -> bool;
    fn isaac_ros2_messages__srv__IsaacPose_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<IsaacPose_Response>);
    fn isaac_ros2_messages__srv__IsaacPose_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<IsaacPose_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<IsaacPose_Response>) -> bool;
}

// Corresponds to isaac_ros2_messages__srv__IsaacPose_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct IsaacPose_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub structure_needs_at_least_one_member: u8,

}



impl Default for IsaacPose_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !isaac_ros2_messages__srv__IsaacPose_Response__init(&mut msg as *mut _) {
        panic!("Call to isaac_ros2_messages__srv__IsaacPose_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for IsaacPose_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { isaac_ros2_messages__srv__IsaacPose_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { isaac_ros2_messages__srv__IsaacPose_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { isaac_ros2_messages__srv__IsaacPose_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for IsaacPose_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for IsaacPose_Response where Self: Sized {
  const TYPE_NAME: &'static str = "isaac_ros2_messages/srv/IsaacPose_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__isaac_ros2_messages__srv__IsaacPose_Response() }
  }
}


#[link(name = "isaac_ros2_messages__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__isaac_ros2_messages__srv__GetPrims_Request() -> *const std::ffi::c_void;
}

#[link(name = "isaac_ros2_messages__rosidl_generator_c")]
extern "C" {
    fn isaac_ros2_messages__srv__GetPrims_Request__init(msg: *mut GetPrims_Request) -> bool;
    fn isaac_ros2_messages__srv__GetPrims_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<GetPrims_Request>, size: usize) -> bool;
    fn isaac_ros2_messages__srv__GetPrims_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<GetPrims_Request>);
    fn isaac_ros2_messages__srv__GetPrims_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<GetPrims_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<GetPrims_Request>) -> bool;
}

// Corresponds to isaac_ros2_messages__srv__GetPrims_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GetPrims_Request {
    /// get prims at path
    pub path: rosidl_runtime_rs::String,

}



impl Default for GetPrims_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !isaac_ros2_messages__srv__GetPrims_Request__init(&mut msg as *mut _) {
        panic!("Call to isaac_ros2_messages__srv__GetPrims_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for GetPrims_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { isaac_ros2_messages__srv__GetPrims_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { isaac_ros2_messages__srv__GetPrims_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { isaac_ros2_messages__srv__GetPrims_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for GetPrims_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for GetPrims_Request where Self: Sized {
  const TYPE_NAME: &'static str = "isaac_ros2_messages/srv/GetPrims_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__isaac_ros2_messages__srv__GetPrims_Request() }
  }
}


#[link(name = "isaac_ros2_messages__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__isaac_ros2_messages__srv__GetPrims_Response() -> *const std::ffi::c_void;
}

#[link(name = "isaac_ros2_messages__rosidl_generator_c")]
extern "C" {
    fn isaac_ros2_messages__srv__GetPrims_Response__init(msg: *mut GetPrims_Response) -> bool;
    fn isaac_ros2_messages__srv__GetPrims_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<GetPrims_Response>, size: usize) -> bool;
    fn isaac_ros2_messages__srv__GetPrims_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<GetPrims_Response>);
    fn isaac_ros2_messages__srv__GetPrims_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<GetPrims_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<GetPrims_Response>) -> bool;
}

// Corresponds to isaac_ros2_messages__srv__GetPrims_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GetPrims_Response {
    /// list of prim paths
    pub paths: rosidl_runtime_rs::Sequence<rosidl_runtime_rs::String>,

    /// prim type names
    pub types: rosidl_runtime_rs::Sequence<rosidl_runtime_rs::String>,

    /// indicate a successful execution of the service
    pub success: bool,

    /// informational, e.g. for error messages
    pub message: rosidl_runtime_rs::String,

}



impl Default for GetPrims_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !isaac_ros2_messages__srv__GetPrims_Response__init(&mut msg as *mut _) {
        panic!("Call to isaac_ros2_messages__srv__GetPrims_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for GetPrims_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { isaac_ros2_messages__srv__GetPrims_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { isaac_ros2_messages__srv__GetPrims_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { isaac_ros2_messages__srv__GetPrims_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for GetPrims_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for GetPrims_Response where Self: Sized {
  const TYPE_NAME: &'static str = "isaac_ros2_messages/srv/GetPrims_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__isaac_ros2_messages__srv__GetPrims_Response() }
  }
}


#[link(name = "isaac_ros2_messages__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__isaac_ros2_messages__srv__GetPrimAttributes_Request() -> *const std::ffi::c_void;
}

#[link(name = "isaac_ros2_messages__rosidl_generator_c")]
extern "C" {
    fn isaac_ros2_messages__srv__GetPrimAttributes_Request__init(msg: *mut GetPrimAttributes_Request) -> bool;
    fn isaac_ros2_messages__srv__GetPrimAttributes_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<GetPrimAttributes_Request>, size: usize) -> bool;
    fn isaac_ros2_messages__srv__GetPrimAttributes_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<GetPrimAttributes_Request>);
    fn isaac_ros2_messages__srv__GetPrimAttributes_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<GetPrimAttributes_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<GetPrimAttributes_Request>) -> bool;
}

// Corresponds to isaac_ros2_messages__srv__GetPrimAttributes_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GetPrimAttributes_Request {
    /// prim path
    pub path: rosidl_runtime_rs::String,

}



impl Default for GetPrimAttributes_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !isaac_ros2_messages__srv__GetPrimAttributes_Request__init(&mut msg as *mut _) {
        panic!("Call to isaac_ros2_messages__srv__GetPrimAttributes_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for GetPrimAttributes_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { isaac_ros2_messages__srv__GetPrimAttributes_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { isaac_ros2_messages__srv__GetPrimAttributes_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { isaac_ros2_messages__srv__GetPrimAttributes_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for GetPrimAttributes_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for GetPrimAttributes_Request where Self: Sized {
  const TYPE_NAME: &'static str = "isaac_ros2_messages/srv/GetPrimAttributes_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__isaac_ros2_messages__srv__GetPrimAttributes_Request() }
  }
}


#[link(name = "isaac_ros2_messages__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__isaac_ros2_messages__srv__GetPrimAttributes_Response() -> *const std::ffi::c_void;
}

#[link(name = "isaac_ros2_messages__rosidl_generator_c")]
extern "C" {
    fn isaac_ros2_messages__srv__GetPrimAttributes_Response__init(msg: *mut GetPrimAttributes_Response) -> bool;
    fn isaac_ros2_messages__srv__GetPrimAttributes_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<GetPrimAttributes_Response>, size: usize) -> bool;
    fn isaac_ros2_messages__srv__GetPrimAttributes_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<GetPrimAttributes_Response>);
    fn isaac_ros2_messages__srv__GetPrimAttributes_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<GetPrimAttributes_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<GetPrimAttributes_Response>) -> bool;
}

// Corresponds to isaac_ros2_messages__srv__GetPrimAttributes_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GetPrimAttributes_Response {
    /// list of attribute base names (name used to Get or Set an attribute)
    pub names: rosidl_runtime_rs::Sequence<rosidl_runtime_rs::String>,

    /// list of attribute display names (name displayed in Property tab)
    pub displays: rosidl_runtime_rs::Sequence<rosidl_runtime_rs::String>,

    /// list of attribute data types
    pub types: rosidl_runtime_rs::Sequence<rosidl_runtime_rs::String>,

    /// indicate a successful execution of the service
    pub success: bool,

    /// informational, e.g. for error messages
    pub message: rosidl_runtime_rs::String,

}



impl Default for GetPrimAttributes_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !isaac_ros2_messages__srv__GetPrimAttributes_Response__init(&mut msg as *mut _) {
        panic!("Call to isaac_ros2_messages__srv__GetPrimAttributes_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for GetPrimAttributes_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { isaac_ros2_messages__srv__GetPrimAttributes_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { isaac_ros2_messages__srv__GetPrimAttributes_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { isaac_ros2_messages__srv__GetPrimAttributes_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for GetPrimAttributes_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for GetPrimAttributes_Response where Self: Sized {
  const TYPE_NAME: &'static str = "isaac_ros2_messages/srv/GetPrimAttributes_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__isaac_ros2_messages__srv__GetPrimAttributes_Response() }
  }
}


#[link(name = "isaac_ros2_messages__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__isaac_ros2_messages__srv__GetPrimAttribute_Request() -> *const std::ffi::c_void;
}

#[link(name = "isaac_ros2_messages__rosidl_generator_c")]
extern "C" {
    fn isaac_ros2_messages__srv__GetPrimAttribute_Request__init(msg: *mut GetPrimAttribute_Request) -> bool;
    fn isaac_ros2_messages__srv__GetPrimAttribute_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<GetPrimAttribute_Request>, size: usize) -> bool;
    fn isaac_ros2_messages__srv__GetPrimAttribute_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<GetPrimAttribute_Request>);
    fn isaac_ros2_messages__srv__GetPrimAttribute_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<GetPrimAttribute_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<GetPrimAttribute_Request>) -> bool;
}

// Corresponds to isaac_ros2_messages__srv__GetPrimAttribute_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GetPrimAttribute_Request {
    /// prim path
    pub path: rosidl_runtime_rs::String,

    /// attribute name
    pub attribute: rosidl_runtime_rs::String,

}



impl Default for GetPrimAttribute_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !isaac_ros2_messages__srv__GetPrimAttribute_Request__init(&mut msg as *mut _) {
        panic!("Call to isaac_ros2_messages__srv__GetPrimAttribute_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for GetPrimAttribute_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { isaac_ros2_messages__srv__GetPrimAttribute_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { isaac_ros2_messages__srv__GetPrimAttribute_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { isaac_ros2_messages__srv__GetPrimAttribute_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for GetPrimAttribute_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for GetPrimAttribute_Request where Self: Sized {
  const TYPE_NAME: &'static str = "isaac_ros2_messages/srv/GetPrimAttribute_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__isaac_ros2_messages__srv__GetPrimAttribute_Request() }
  }
}


#[link(name = "isaac_ros2_messages__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__isaac_ros2_messages__srv__GetPrimAttribute_Response() -> *const std::ffi::c_void;
}

#[link(name = "isaac_ros2_messages__rosidl_generator_c")]
extern "C" {
    fn isaac_ros2_messages__srv__GetPrimAttribute_Response__init(msg: *mut GetPrimAttribute_Response) -> bool;
    fn isaac_ros2_messages__srv__GetPrimAttribute_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<GetPrimAttribute_Response>, size: usize) -> bool;
    fn isaac_ros2_messages__srv__GetPrimAttribute_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<GetPrimAttribute_Response>);
    fn isaac_ros2_messages__srv__GetPrimAttribute_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<GetPrimAttribute_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<GetPrimAttribute_Response>) -> bool;
}

// Corresponds to isaac_ros2_messages__srv__GetPrimAttribute_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GetPrimAttribute_Response {
    /// attribute value (as JSON)
    pub value: rosidl_runtime_rs::String,

    /// attribute type
    pub type_: rosidl_runtime_rs::String,

    /// indicate a successful execution of the service
    pub success: bool,

    /// informational, e.g. for error messages
    pub message: rosidl_runtime_rs::String,

}



impl Default for GetPrimAttribute_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !isaac_ros2_messages__srv__GetPrimAttribute_Response__init(&mut msg as *mut _) {
        panic!("Call to isaac_ros2_messages__srv__GetPrimAttribute_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for GetPrimAttribute_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { isaac_ros2_messages__srv__GetPrimAttribute_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { isaac_ros2_messages__srv__GetPrimAttribute_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { isaac_ros2_messages__srv__GetPrimAttribute_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for GetPrimAttribute_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for GetPrimAttribute_Response where Self: Sized {
  const TYPE_NAME: &'static str = "isaac_ros2_messages/srv/GetPrimAttribute_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__isaac_ros2_messages__srv__GetPrimAttribute_Response() }
  }
}


#[link(name = "isaac_ros2_messages__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__isaac_ros2_messages__srv__SetPrimAttribute_Request() -> *const std::ffi::c_void;
}

#[link(name = "isaac_ros2_messages__rosidl_generator_c")]
extern "C" {
    fn isaac_ros2_messages__srv__SetPrimAttribute_Request__init(msg: *mut SetPrimAttribute_Request) -> bool;
    fn isaac_ros2_messages__srv__SetPrimAttribute_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<SetPrimAttribute_Request>, size: usize) -> bool;
    fn isaac_ros2_messages__srv__SetPrimAttribute_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<SetPrimAttribute_Request>);
    fn isaac_ros2_messages__srv__SetPrimAttribute_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<SetPrimAttribute_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<SetPrimAttribute_Request>) -> bool;
}

// Corresponds to isaac_ros2_messages__srv__SetPrimAttribute_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SetPrimAttribute_Request {
    /// prim path
    pub path: rosidl_runtime_rs::String,

    /// attribute name
    pub attribute: rosidl_runtime_rs::String,

    /// attribute value (as JSON)
    pub value: rosidl_runtime_rs::String,

}



impl Default for SetPrimAttribute_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !isaac_ros2_messages__srv__SetPrimAttribute_Request__init(&mut msg as *mut _) {
        panic!("Call to isaac_ros2_messages__srv__SetPrimAttribute_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for SetPrimAttribute_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { isaac_ros2_messages__srv__SetPrimAttribute_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { isaac_ros2_messages__srv__SetPrimAttribute_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { isaac_ros2_messages__srv__SetPrimAttribute_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for SetPrimAttribute_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for SetPrimAttribute_Request where Self: Sized {
  const TYPE_NAME: &'static str = "isaac_ros2_messages/srv/SetPrimAttribute_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__isaac_ros2_messages__srv__SetPrimAttribute_Request() }
  }
}


#[link(name = "isaac_ros2_messages__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__isaac_ros2_messages__srv__SetPrimAttribute_Response() -> *const std::ffi::c_void;
}

#[link(name = "isaac_ros2_messages__rosidl_generator_c")]
extern "C" {
    fn isaac_ros2_messages__srv__SetPrimAttribute_Response__init(msg: *mut SetPrimAttribute_Response) -> bool;
    fn isaac_ros2_messages__srv__SetPrimAttribute_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<SetPrimAttribute_Response>, size: usize) -> bool;
    fn isaac_ros2_messages__srv__SetPrimAttribute_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<SetPrimAttribute_Response>);
    fn isaac_ros2_messages__srv__SetPrimAttribute_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<SetPrimAttribute_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<SetPrimAttribute_Response>) -> bool;
}

// Corresponds to isaac_ros2_messages__srv__SetPrimAttribute_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SetPrimAttribute_Response {
    /// indicate a successful execution of the service
    pub success: bool,

    /// informational, e.g. for error messages
    pub message: rosidl_runtime_rs::String,

}



impl Default for SetPrimAttribute_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !isaac_ros2_messages__srv__SetPrimAttribute_Response__init(&mut msg as *mut _) {
        panic!("Call to isaac_ros2_messages__srv__SetPrimAttribute_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for SetPrimAttribute_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { isaac_ros2_messages__srv__SetPrimAttribute_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { isaac_ros2_messages__srv__SetPrimAttribute_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { isaac_ros2_messages__srv__SetPrimAttribute_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for SetPrimAttribute_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for SetPrimAttribute_Response where Self: Sized {
  const TYPE_NAME: &'static str = "isaac_ros2_messages/srv/SetPrimAttribute_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__isaac_ros2_messages__srv__SetPrimAttribute_Response() }
  }
}






#[link(name = "isaac_ros2_messages__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__isaac_ros2_messages__srv__IsaacPose() -> *const std::ffi::c_void;
}

// Corresponds to isaac_ros2_messages__srv__IsaacPose
#[allow(missing_docs, non_camel_case_types)]
pub struct IsaacPose;

impl rosidl_runtime_rs::Service for IsaacPose {
    type Request = IsaacPose_Request;
    type Response = IsaacPose_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__isaac_ros2_messages__srv__IsaacPose() }
    }
}




#[link(name = "isaac_ros2_messages__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__isaac_ros2_messages__srv__GetPrims() -> *const std::ffi::c_void;
}

// Corresponds to isaac_ros2_messages__srv__GetPrims
#[allow(missing_docs, non_camel_case_types)]
pub struct GetPrims;

impl rosidl_runtime_rs::Service for GetPrims {
    type Request = GetPrims_Request;
    type Response = GetPrims_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__isaac_ros2_messages__srv__GetPrims() }
    }
}




#[link(name = "isaac_ros2_messages__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__isaac_ros2_messages__srv__GetPrimAttributes() -> *const std::ffi::c_void;
}

// Corresponds to isaac_ros2_messages__srv__GetPrimAttributes
#[allow(missing_docs, non_camel_case_types)]
pub struct GetPrimAttributes;

impl rosidl_runtime_rs::Service for GetPrimAttributes {
    type Request = GetPrimAttributes_Request;
    type Response = GetPrimAttributes_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__isaac_ros2_messages__srv__GetPrimAttributes() }
    }
}




#[link(name = "isaac_ros2_messages__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__isaac_ros2_messages__srv__GetPrimAttribute() -> *const std::ffi::c_void;
}

// Corresponds to isaac_ros2_messages__srv__GetPrimAttribute
#[allow(missing_docs, non_camel_case_types)]
pub struct GetPrimAttribute;

impl rosidl_runtime_rs::Service for GetPrimAttribute {
    type Request = GetPrimAttribute_Request;
    type Response = GetPrimAttribute_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__isaac_ros2_messages__srv__GetPrimAttribute() }
    }
}




#[link(name = "isaac_ros2_messages__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__isaac_ros2_messages__srv__SetPrimAttribute() -> *const std::ffi::c_void;
}

// Corresponds to isaac_ros2_messages__srv__SetPrimAttribute
#[allow(missing_docs, non_camel_case_types)]
pub struct SetPrimAttribute;

impl rosidl_runtime_rs::Service for SetPrimAttribute {
    type Request = SetPrimAttribute_Request;
    type Response = SetPrimAttribute_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__isaac_ros2_messages__srv__SetPrimAttribute() }
    }
}


