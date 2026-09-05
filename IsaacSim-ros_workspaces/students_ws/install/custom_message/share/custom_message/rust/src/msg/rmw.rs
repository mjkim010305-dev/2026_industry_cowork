#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};


#[link(name = "custom_message__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__custom_message__msg__SampleMsg() -> *const std::ffi::c_void;
}

#[link(name = "custom_message__rosidl_generator_c")]
extern "C" {
    fn custom_message__msg__SampleMsg__init(msg: *mut SampleMsg) -> bool;
    fn custom_message__msg__SampleMsg__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<SampleMsg>, size: usize) -> bool;
    fn custom_message__msg__SampleMsg__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<SampleMsg>);
    fn custom_message__msg__SampleMsg__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<SampleMsg>, out_seq: *mut rosidl_runtime_rs::Sequence<SampleMsg>) -> bool;
}

// Corresponds to custom_message__msg__SampleMsg
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SampleMsg {

    // This member is not documented.
    #[allow(missing_docs)]
    pub my_string: std_msgs::msg::rmw::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub my_num: i64,

}



impl Default for SampleMsg {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !custom_message__msg__SampleMsg__init(&mut msg as *mut _) {
        panic!("Call to custom_message__msg__SampleMsg__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for SampleMsg {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { custom_message__msg__SampleMsg__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { custom_message__msg__SampleMsg__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { custom_message__msg__SampleMsg__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for SampleMsg {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for SampleMsg where Self: Sized {
  const TYPE_NAME: &'static str = "custom_message/msg/SampleMsg";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__custom_message__msg__SampleMsg() }
  }
}


