#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};


#[link(name = "perception_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__perception_interfaces__msg__DetectedObject() -> *const std::ffi::c_void;
}

#[link(name = "perception_interfaces__rosidl_generator_c")]
extern "C" {
    fn perception_interfaces__msg__DetectedObject__init(msg: *mut DetectedObject) -> bool;
    fn perception_interfaces__msg__DetectedObject__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<DetectedObject>, size: usize) -> bool;
    fn perception_interfaces__msg__DetectedObject__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<DetectedObject>);
    fn perception_interfaces__msg__DetectedObject__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<DetectedObject>, out_seq: *mut rosidl_runtime_rs::Sequence<DetectedObject>) -> bool;
}

// Corresponds to perception_interfaces__msg__DetectedObject
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// 9단계: 물체 하나에 대한 정보.
/// notes/message_spec.md v3 기준 — class_name/orientation은 아직 추가하지 않음(팀원2 요청, 후순위 보류).

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DetectedObject {
    /// 프레임 내 순번(0,1,2,...). 프레임 간 추적 ID 아님 — 열린 이슈 참고.
    pub id: i32,

    /// base_link 기준 (x, y, z), 단위 m
    pub position: geometry_msgs::msg::rmw::Point,

    /// size.x=width(폭), size.y=depth(깊이), size.z=height(높이), 단위 m
    /// (Vector3에는 w/d/h라는 이름이 없어서 x/y/z에 이 순서로 대응시킨다)
    pub size: geometry_msgs::msg::rmw::Vector3,

}



impl Default for DetectedObject {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !perception_interfaces__msg__DetectedObject__init(&mut msg as *mut _) {
        panic!("Call to perception_interfaces__msg__DetectedObject__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for DetectedObject {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { perception_interfaces__msg__DetectedObject__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { perception_interfaces__msg__DetectedObject__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { perception_interfaces__msg__DetectedObject__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for DetectedObject {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for DetectedObject where Self: Sized {
  const TYPE_NAME: &'static str = "perception_interfaces/msg/DetectedObject";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__perception_interfaces__msg__DetectedObject() }
  }
}


#[link(name = "perception_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__perception_interfaces__msg__DetectedObjectArray() -> *const std::ffi::c_void;
}

#[link(name = "perception_interfaces__rosidl_generator_c")]
extern "C" {
    fn perception_interfaces__msg__DetectedObjectArray__init(msg: *mut DetectedObjectArray) -> bool;
    fn perception_interfaces__msg__DetectedObjectArray__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<DetectedObjectArray>, size: usize) -> bool;
    fn perception_interfaces__msg__DetectedObjectArray__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<DetectedObjectArray>);
    fn perception_interfaces__msg__DetectedObjectArray__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<DetectedObjectArray>, out_seq: *mut rosidl_runtime_rs::Sequence<DetectedObjectArray>) -> bool;
}

// Corresponds to perception_interfaces__msg__DetectedObjectArray
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// 9단계: 이 노드가 실제로 /detected_object 토픽에 publish하는 메시지.
/// notes/message_spec.md v3 기준 — header.frame_id는 "base_link" 고정,
/// detected=false일 때는 objects가 빈 배열이어야 한다(토픽 자체는 생략하지 않음).

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DetectedObjectArray {
    /// header.frame_id = "base_link"
    pub header: std_msgs::msg::rmw::Header,

    /// 이번 프레임에 검출된 물체가 하나라도 있으면 true
    pub detected: bool,

    /// 검출된 물체 목록. 원소가 1개뿐이어도 항상 배열 형태.
    pub objects: rosidl_runtime_rs::Sequence<super::super::msg::rmw::DetectedObject>,

}



impl Default for DetectedObjectArray {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !perception_interfaces__msg__DetectedObjectArray__init(&mut msg as *mut _) {
        panic!("Call to perception_interfaces__msg__DetectedObjectArray__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for DetectedObjectArray {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { perception_interfaces__msg__DetectedObjectArray__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { perception_interfaces__msg__DetectedObjectArray__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { perception_interfaces__msg__DetectedObjectArray__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for DetectedObjectArray {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for DetectedObjectArray where Self: Sized {
  const TYPE_NAME: &'static str = "perception_interfaces/msg/DetectedObjectArray";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__perception_interfaces__msg__DetectedObjectArray() }
  }
}


