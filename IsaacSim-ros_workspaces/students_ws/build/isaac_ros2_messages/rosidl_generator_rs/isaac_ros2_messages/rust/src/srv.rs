#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};




// Corresponds to isaac_ros2_messages__srv__IsaacPose_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct IsaacPose_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub names: Vec<std::string::String>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub poses: Vec<geometry_msgs::msg::Pose>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub velocities: Vec<geometry_msgs::msg::Twist>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub scales: Vec<geometry_msgs::msg::Vector3>,

}



impl Default for IsaacPose_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::IsaacPose_Request::default())
  }
}

impl rosidl_runtime_rs::Message for IsaacPose_Request {
  type RmwMsg = super::srv::rmw::IsaacPose_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        names: msg.names
          .into_iter()
          .map(|elem| elem.as_str().into())
          .collect(),
        poses: msg.poses
          .into_iter()
          .map(|elem| geometry_msgs::msg::Pose::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
        velocities: msg.velocities
          .into_iter()
          .map(|elem| geometry_msgs::msg::Twist::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
        scales: msg.scales
          .into_iter()
          .map(|elem| geometry_msgs::msg::Vector3::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
        names: msg.names
          .iter()
          .map(|elem| elem.as_str().into())
          .collect(),
        poses: msg.poses
          .iter()
          .map(|elem| geometry_msgs::msg::Pose::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
        velocities: msg.velocities
          .iter()
          .map(|elem| geometry_msgs::msg::Twist::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
        scales: msg.scales
          .iter()
          .map(|elem| geometry_msgs::msg::Vector3::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      names: msg.names
          .into_iter()
          .map(|elem| elem.to_string())
          .collect(),
      poses: msg.poses
          .into_iter()
          .map(geometry_msgs::msg::Pose::from_rmw_message)
          .collect(),
      velocities: msg.velocities
          .into_iter()
          .map(geometry_msgs::msg::Twist::from_rmw_message)
          .collect(),
      scales: msg.scales
          .into_iter()
          .map(geometry_msgs::msg::Vector3::from_rmw_message)
          .collect(),
    }
  }
}


// Corresponds to isaac_ros2_messages__srv__IsaacPose_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct IsaacPose_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub structure_needs_at_least_one_member: u8,

}



impl Default for IsaacPose_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::IsaacPose_Response::default())
  }
}

impl rosidl_runtime_rs::Message for IsaacPose_Response {
  type RmwMsg = super::srv::rmw::IsaacPose_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        structure_needs_at_least_one_member: msg.structure_needs_at_least_one_member,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      structure_needs_at_least_one_member: msg.structure_needs_at_least_one_member,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      structure_needs_at_least_one_member: msg.structure_needs_at_least_one_member,
    }
  }
}


// Corresponds to isaac_ros2_messages__srv__GetPrims_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GetPrims_Request {
    /// get prims at path
    pub path: std::string::String,

}



impl Default for GetPrims_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::GetPrims_Request::default())
  }
}

impl rosidl_runtime_rs::Message for GetPrims_Request {
  type RmwMsg = super::srv::rmw::GetPrims_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        path: msg.path.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        path: msg.path.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      path: msg.path.to_string(),
    }
  }
}


// Corresponds to isaac_ros2_messages__srv__GetPrims_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GetPrims_Response {
    /// list of prim paths
    pub paths: Vec<std::string::String>,

    /// prim type names
    pub types: Vec<std::string::String>,

    /// indicate a successful execution of the service
    pub success: bool,

    /// informational, e.g. for error messages
    pub message: std::string::String,

}



impl Default for GetPrims_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::GetPrims_Response::default())
  }
}

impl rosidl_runtime_rs::Message for GetPrims_Response {
  type RmwMsg = super::srv::rmw::GetPrims_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        paths: msg.paths
          .into_iter()
          .map(|elem| elem.as_str().into())
          .collect(),
        types: msg.types
          .into_iter()
          .map(|elem| elem.as_str().into())
          .collect(),
        success: msg.success,
        message: msg.message.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        paths: msg.paths
          .iter()
          .map(|elem| elem.as_str().into())
          .collect(),
        types: msg.types
          .iter()
          .map(|elem| elem.as_str().into())
          .collect(),
      success: msg.success,
        message: msg.message.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      paths: msg.paths
          .into_iter()
          .map(|elem| elem.to_string())
          .collect(),
      types: msg.types
          .into_iter()
          .map(|elem| elem.to_string())
          .collect(),
      success: msg.success,
      message: msg.message.to_string(),
    }
  }
}


// Corresponds to isaac_ros2_messages__srv__GetPrimAttributes_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GetPrimAttributes_Request {
    /// prim path
    pub path: std::string::String,

}



impl Default for GetPrimAttributes_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::GetPrimAttributes_Request::default())
  }
}

impl rosidl_runtime_rs::Message for GetPrimAttributes_Request {
  type RmwMsg = super::srv::rmw::GetPrimAttributes_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        path: msg.path.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        path: msg.path.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      path: msg.path.to_string(),
    }
  }
}


// Corresponds to isaac_ros2_messages__srv__GetPrimAttributes_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GetPrimAttributes_Response {
    /// list of attribute base names (name used to Get or Set an attribute)
    pub names: Vec<std::string::String>,

    /// list of attribute display names (name displayed in Property tab)
    pub displays: Vec<std::string::String>,

    /// list of attribute data types
    pub types: Vec<std::string::String>,

    /// indicate a successful execution of the service
    pub success: bool,

    /// informational, e.g. for error messages
    pub message: std::string::String,

}



impl Default for GetPrimAttributes_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::GetPrimAttributes_Response::default())
  }
}

impl rosidl_runtime_rs::Message for GetPrimAttributes_Response {
  type RmwMsg = super::srv::rmw::GetPrimAttributes_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        names: msg.names
          .into_iter()
          .map(|elem| elem.as_str().into())
          .collect(),
        displays: msg.displays
          .into_iter()
          .map(|elem| elem.as_str().into())
          .collect(),
        types: msg.types
          .into_iter()
          .map(|elem| elem.as_str().into())
          .collect(),
        success: msg.success,
        message: msg.message.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        names: msg.names
          .iter()
          .map(|elem| elem.as_str().into())
          .collect(),
        displays: msg.displays
          .iter()
          .map(|elem| elem.as_str().into())
          .collect(),
        types: msg.types
          .iter()
          .map(|elem| elem.as_str().into())
          .collect(),
      success: msg.success,
        message: msg.message.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      names: msg.names
          .into_iter()
          .map(|elem| elem.to_string())
          .collect(),
      displays: msg.displays
          .into_iter()
          .map(|elem| elem.to_string())
          .collect(),
      types: msg.types
          .into_iter()
          .map(|elem| elem.to_string())
          .collect(),
      success: msg.success,
      message: msg.message.to_string(),
    }
  }
}


// Corresponds to isaac_ros2_messages__srv__GetPrimAttribute_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GetPrimAttribute_Request {
    /// prim path
    pub path: std::string::String,

    /// attribute name
    pub attribute: std::string::String,

}



impl Default for GetPrimAttribute_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::GetPrimAttribute_Request::default())
  }
}

impl rosidl_runtime_rs::Message for GetPrimAttribute_Request {
  type RmwMsg = super::srv::rmw::GetPrimAttribute_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        path: msg.path.as_str().into(),
        attribute: msg.attribute.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        path: msg.path.as_str().into(),
        attribute: msg.attribute.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      path: msg.path.to_string(),
      attribute: msg.attribute.to_string(),
    }
  }
}


// Corresponds to isaac_ros2_messages__srv__GetPrimAttribute_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GetPrimAttribute_Response {
    /// attribute value (as JSON)
    pub value: std::string::String,

    /// attribute type
    pub type_: std::string::String,

    /// indicate a successful execution of the service
    pub success: bool,

    /// informational, e.g. for error messages
    pub message: std::string::String,

}



impl Default for GetPrimAttribute_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::GetPrimAttribute_Response::default())
  }
}

impl rosidl_runtime_rs::Message for GetPrimAttribute_Response {
  type RmwMsg = super::srv::rmw::GetPrimAttribute_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        value: msg.value.as_str().into(),
        type_: msg.type_.as_str().into(),
        success: msg.success,
        message: msg.message.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        value: msg.value.as_str().into(),
        type_: msg.type_.as_str().into(),
      success: msg.success,
        message: msg.message.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      value: msg.value.to_string(),
      type_: msg.type_.to_string(),
      success: msg.success,
      message: msg.message.to_string(),
    }
  }
}


// Corresponds to isaac_ros2_messages__srv__SetPrimAttribute_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SetPrimAttribute_Request {
    /// prim path
    pub path: std::string::String,

    /// attribute name
    pub attribute: std::string::String,

    /// attribute value (as JSON)
    pub value: std::string::String,

}



impl Default for SetPrimAttribute_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::SetPrimAttribute_Request::default())
  }
}

impl rosidl_runtime_rs::Message for SetPrimAttribute_Request {
  type RmwMsg = super::srv::rmw::SetPrimAttribute_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        path: msg.path.as_str().into(),
        attribute: msg.attribute.as_str().into(),
        value: msg.value.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        path: msg.path.as_str().into(),
        attribute: msg.attribute.as_str().into(),
        value: msg.value.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      path: msg.path.to_string(),
      attribute: msg.attribute.to_string(),
      value: msg.value.to_string(),
    }
  }
}


// Corresponds to isaac_ros2_messages__srv__SetPrimAttribute_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SetPrimAttribute_Response {
    /// indicate a successful execution of the service
    pub success: bool,

    /// informational, e.g. for error messages
    pub message: std::string::String,

}



impl Default for SetPrimAttribute_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::SetPrimAttribute_Response::default())
  }
}

impl rosidl_runtime_rs::Message for SetPrimAttribute_Response {
  type RmwMsg = super::srv::rmw::SetPrimAttribute_Response;

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


