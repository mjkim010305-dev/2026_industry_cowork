// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from perception_interfaces:msg/DetectedObjectArray.idl
// generated code does not contain a copyright notice

#ifndef PERCEPTION_INTERFACES__MSG__DETAIL__DETECTED_OBJECT_ARRAY__STRUCT_H_
#define PERCEPTION_INTERFACES__MSG__DETAIL__DETECTED_OBJECT_ARRAY__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'objects'
#include "perception_interfaces/msg/detail/detected_object__struct.h"

/// Struct defined in msg/DetectedObjectArray in the package perception_interfaces.
/**
  * 9단계: 이 노드가 실제로 /detected_object 토픽에 publish하는 메시지.
  * notes/message_spec.md v3 기준 — header.frame_id는 "base_link" 고정,
  * detected=false일 때는 objects가 빈 배열이어야 한다(토픽 자체는 생략하지 않음).
 */
typedef struct perception_interfaces__msg__DetectedObjectArray
{
  /// header.frame_id = "base_link"
  std_msgs__msg__Header header;
  /// 이번 프레임에 검출된 물체가 하나라도 있으면 true
  bool detected;
  /// 검출된 물체 목록. 원소가 1개뿐이어도 항상 배열 형태.
  perception_interfaces__msg__DetectedObject__Sequence objects;
} perception_interfaces__msg__DetectedObjectArray;

// Struct for a sequence of perception_interfaces__msg__DetectedObjectArray.
typedef struct perception_interfaces__msg__DetectedObjectArray__Sequence
{
  perception_interfaces__msg__DetectedObjectArray * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} perception_interfaces__msg__DetectedObjectArray__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PERCEPTION_INTERFACES__MSG__DETAIL__DETECTED_OBJECT_ARRAY__STRUCT_H_
