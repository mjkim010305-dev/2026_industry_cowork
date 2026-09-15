// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from perception_interfaces:msg/DetectedObject.idl
// generated code does not contain a copyright notice

#ifndef PERCEPTION_INTERFACES__MSG__DETAIL__DETECTED_OBJECT__STRUCT_H_
#define PERCEPTION_INTERFACES__MSG__DETAIL__DETECTED_OBJECT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'position'
#include "geometry_msgs/msg/detail/point__struct.h"
// Member 'size'
#include "geometry_msgs/msg/detail/vector3__struct.h"

/// Struct defined in msg/DetectedObject in the package perception_interfaces.
/**
  * 9단계: 물체 하나에 대한 정보.
  * notes/message_spec.md v3 기준 — class_name/orientation은 아직 추가하지 않음(팀원2 요청, 후순위 보류).
 */
typedef struct perception_interfaces__msg__DetectedObject
{
  /// 프레임 내 순번(0,1,2,...). 프레임 간 추적 ID 아님 — 열린 이슈 참고.
  int32_t id;
  /// base_link 기준 (x, y, z), 단위 m
  geometry_msgs__msg__Point position;
  /// size.x=width(폭), size.y=depth(깊이), size.z=height(높이), 단위 m
  /// (Vector3에는 w/d/h라는 이름이 없어서 x/y/z에 이 순서로 대응시킨다)
  geometry_msgs__msg__Vector3 size;
} perception_interfaces__msg__DetectedObject;

// Struct for a sequence of perception_interfaces__msg__DetectedObject.
typedef struct perception_interfaces__msg__DetectedObject__Sequence
{
  perception_interfaces__msg__DetectedObject * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} perception_interfaces__msg__DetectedObject__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PERCEPTION_INTERFACES__MSG__DETAIL__DETECTED_OBJECT__STRUCT_H_
