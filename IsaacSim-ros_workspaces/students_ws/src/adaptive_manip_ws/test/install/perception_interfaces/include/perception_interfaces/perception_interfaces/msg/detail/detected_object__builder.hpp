// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from perception_interfaces:msg/DetectedObject.idl
// generated code does not contain a copyright notice

#ifndef PERCEPTION_INTERFACES__MSG__DETAIL__DETECTED_OBJECT__BUILDER_HPP_
#define PERCEPTION_INTERFACES__MSG__DETAIL__DETECTED_OBJECT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "perception_interfaces/msg/detail/detected_object__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace perception_interfaces
{

namespace msg
{

namespace builder
{

class Init_DetectedObject_size
{
public:
  explicit Init_DetectedObject_size(::perception_interfaces::msg::DetectedObject & msg)
  : msg_(msg)
  {}
  ::perception_interfaces::msg::DetectedObject size(::perception_interfaces::msg::DetectedObject::_size_type arg)
  {
    msg_.size = std::move(arg);
    return std::move(msg_);
  }

private:
  ::perception_interfaces::msg::DetectedObject msg_;
};

class Init_DetectedObject_position
{
public:
  explicit Init_DetectedObject_position(::perception_interfaces::msg::DetectedObject & msg)
  : msg_(msg)
  {}
  Init_DetectedObject_size position(::perception_interfaces::msg::DetectedObject::_position_type arg)
  {
    msg_.position = std::move(arg);
    return Init_DetectedObject_size(msg_);
  }

private:
  ::perception_interfaces::msg::DetectedObject msg_;
};

class Init_DetectedObject_id
{
public:
  Init_DetectedObject_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DetectedObject_position id(::perception_interfaces::msg::DetectedObject::_id_type arg)
  {
    msg_.id = std::move(arg);
    return Init_DetectedObject_position(msg_);
  }

private:
  ::perception_interfaces::msg::DetectedObject msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::perception_interfaces::msg::DetectedObject>()
{
  return perception_interfaces::msg::builder::Init_DetectedObject_id();
}

}  // namespace perception_interfaces

#endif  // PERCEPTION_INTERFACES__MSG__DETAIL__DETECTED_OBJECT__BUILDER_HPP_
