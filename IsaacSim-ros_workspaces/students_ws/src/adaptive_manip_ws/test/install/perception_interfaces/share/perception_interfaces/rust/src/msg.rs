#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



// Corresponds to perception_interfaces__msg__DetectedObject
/// 9단계: 물체 하나에 대한 정보.
/// notes/message_spec.md v3 기준 — class_name/orientation은 아직 추가하지 않음(팀원2 요청, 후순위 보류).

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DetectedObject {
    /// 프레임 내 순번(0,1,2,...). 프레임 간 추적 ID 아님 — 열린 이슈 참고.
    pub id: i32,

    /// base_link 기준 (x, y, z), 단위 m
    pub position: geometry_msgs::msg::Point,

    /// size.x=width(폭), size.y=depth(깊이), size.z=height(높이), 단위 m
    /// (Vector3에는 w/d/h라는 이름이 없어서 x/y/z에 이 순서로 대응시킨다)
    pub size: geometry_msgs::msg::Vector3,

}



impl Default for DetectedObject {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::DetectedObject::default())
  }
}

impl rosidl_runtime_rs::Message for DetectedObject {
  type RmwMsg = super::msg::rmw::DetectedObject;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        id: msg.id,
        position: geometry_msgs::msg::Point::into_rmw_message(std::borrow::Cow::Owned(msg.position)).into_owned(),
        size: geometry_msgs::msg::Vector3::into_rmw_message(std::borrow::Cow::Owned(msg.size)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      id: msg.id,
        position: geometry_msgs::msg::Point::into_rmw_message(std::borrow::Cow::Borrowed(&msg.position)).into_owned(),
        size: geometry_msgs::msg::Vector3::into_rmw_message(std::borrow::Cow::Borrowed(&msg.size)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      id: msg.id,
      position: geometry_msgs::msg::Point::from_rmw_message(msg.position),
      size: geometry_msgs::msg::Vector3::from_rmw_message(msg.size),
    }
  }
}


// Corresponds to perception_interfaces__msg__DetectedObjectArray
/// 9단계: 이 노드가 실제로 /detected_object 토픽에 publish하는 메시지.
/// notes/message_spec.md v3 기준 — header.frame_id는 "base_link" 고정,
/// detected=false일 때는 objects가 빈 배열이어야 한다(토픽 자체는 생략하지 않음).

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct DetectedObjectArray {
    /// header.frame_id = "base_link"
    pub header: std_msgs::msg::Header,

    /// 이번 프레임에 검출된 물체가 하나라도 있으면 true
    pub detected: bool,

    /// 검출된 물체 목록. 원소가 1개뿐이어도 항상 배열 형태.
    pub objects: Vec<super::msg::DetectedObject>,

}



impl Default for DetectedObjectArray {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::DetectedObjectArray::default())
  }
}

impl rosidl_runtime_rs::Message for DetectedObjectArray {
  type RmwMsg = super::msg::rmw::DetectedObjectArray;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        detected: msg.detected,
        objects: msg.objects
          .into_iter()
          .map(|elem| super::msg::DetectedObject::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
      detected: msg.detected,
        objects: msg.objects
          .iter()
          .map(|elem| super::msg::DetectedObject::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      detected: msg.detected,
      objects: msg.objects
          .into_iter()
          .map(super::msg::DetectedObject::from_rmw_message)
          .collect(),
    }
  }
}


