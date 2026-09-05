#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



// Corresponds to custom_message__msg__SampleMsg

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SampleMsg {

    // This member is not documented.
    #[allow(missing_docs)]
    pub my_string: std_msgs::msg::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub my_num: i64,

}



impl Default for SampleMsg {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::SampleMsg::default())
  }
}

impl rosidl_runtime_rs::Message for SampleMsg {
  type RmwMsg = super::msg::rmw::SampleMsg;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        my_string: std_msgs::msg::String::into_rmw_message(std::borrow::Cow::Owned(msg.my_string)).into_owned(),
        my_num: msg.my_num,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        my_string: std_msgs::msg::String::into_rmw_message(std::borrow::Cow::Borrowed(&msg.my_string)).into_owned(),
      my_num: msg.my_num,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      my_string: std_msgs::msg::String::from_rmw_message(msg.my_string),
      my_num: msg.my_num,
    }
  }
}


