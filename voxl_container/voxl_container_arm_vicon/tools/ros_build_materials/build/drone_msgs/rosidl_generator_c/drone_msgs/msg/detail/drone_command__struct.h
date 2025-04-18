// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from drone_msgs:msg/DroneCommand.idl
// generated code does not contain a copyright notice

#ifndef DRONE_MSGS__MSG__DETAIL__DRONE_COMMAND__STRUCT_H_
#define DRONE_MSGS__MSG__DETAIL__DRONE_COMMAND__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'command'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/DroneCommand in the package drone_msgs.
typedef struct drone_msgs__msg__DroneCommand
{
  rosidl_runtime_c__String command;
  float position[3];
} drone_msgs__msg__DroneCommand;

// Struct for a sequence of drone_msgs__msg__DroneCommand.
typedef struct drone_msgs__msg__DroneCommand__Sequence
{
  drone_msgs__msg__DroneCommand * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} drone_msgs__msg__DroneCommand__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // DRONE_MSGS__MSG__DETAIL__DRONE_COMMAND__STRUCT_H_
