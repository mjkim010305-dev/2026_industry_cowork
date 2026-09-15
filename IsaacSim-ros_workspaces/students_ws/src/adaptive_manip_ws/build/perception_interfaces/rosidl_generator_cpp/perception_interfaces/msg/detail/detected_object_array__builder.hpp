// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from perception_interfaces:msg/DetectedObjectArray.idl
// generated code does not contain a copyright notice

#ifndef PERCEPTION_INTERFACES__MSG__DETAIL__DETECTED_OBJECT_ARRAY__BUILDER_HPP_
#define PERCEPTION_INTERFACES__MSG__DETAIL__DETECTED_OBJECT_ARRAY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "perception_interfaces/msg/detail/detected_object_array__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace perception_interfaces
{

namespace msg
{

namespace builder
{

class Init_DetectedObjectArray_objects
{
public:
  explicit Init_DetectedObjectArray_objects(::perception_interfaces::msg::DetectedObjectArray & msg)
  : msg_(msg)
  {}
  ::perception_interfaces::msg::DetectedObjectArray objects(::perception_interfaces::msg::DetectedObjectArray::_objects_type arg)
  {
    msg_.objects = std::move(arg);
    return std::move(msg_);
  }

private:
  ::perception_interfaces::msg::DetectedObjectArray msg_;
};

class Init_DetectedObjectArray_detected
{
public:
  explicit Init_DetectedObjectArray_detected(::perception_interfaces::msg::DetectedObjectArray & msg)
  : msg_(msg)
  {}
  Init_DetectedObjectArray_objects detected(::perception_interfaces::msg::DetectedObjectArray::_detected_type arg)
  {
    msg_.detected = std::move(arg);
    return Init_DetectedObjectArray_objects(msg_);
  }

private:
  ::perception_interfaces::msg::DetectedObjectArray msg_;
};

class Init_DetectedObjectArray_header
{
public:
  Init_DetectedObjectArray_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DetectedObjectArray_detected header(::perception_interfaces::msg::DetectedObjectArray::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_DetectedObjectArray_detected(msg_);
  }

private:
  ::perception_interfaces::msg::DetectedObjectArray msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::perception_interfaces::msg::DetectedObjectArray>()
{
  return perception_interfaces::msg::builder::Init_DetectedObjectArray_header();
}

}  // namespace perception_interfaces

#endif  // PERCEPTION_INTERFACES__MSG__DETAIL__DETECTED_OBJECT_ARRAY__BUILDER_HPP_
