// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from drone_msgs:msg/DroneCommand.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "drone_msgs/msg/detail/drone_command__rosidl_typesupport_introspection_c.h"
#include "drone_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "drone_msgs/msg/detail/drone_command__functions.h"
#include "drone_msgs/msg/detail/drone_command__struct.h"


// Include directives for member types
// Member `command`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void drone_msgs__msg__DroneCommand__rosidl_typesupport_introspection_c__DroneCommand_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  drone_msgs__msg__DroneCommand__init(message_memory);
}

void drone_msgs__msg__DroneCommand__rosidl_typesupport_introspection_c__DroneCommand_fini_function(void * message_memory)
{
  drone_msgs__msg__DroneCommand__fini(message_memory);
}

size_t drone_msgs__msg__DroneCommand__rosidl_typesupport_introspection_c__size_function__DroneCommand__position(
  const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * drone_msgs__msg__DroneCommand__rosidl_typesupport_introspection_c__get_const_function__DroneCommand__position(
  const void * untyped_member, size_t index)
{
  const float * member =
    (const float *)(untyped_member);
  return &member[index];
}

void * drone_msgs__msg__DroneCommand__rosidl_typesupport_introspection_c__get_function__DroneCommand__position(
  void * untyped_member, size_t index)
{
  float * member =
    (float *)(untyped_member);
  return &member[index];
}

void drone_msgs__msg__DroneCommand__rosidl_typesupport_introspection_c__fetch_function__DroneCommand__position(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    drone_msgs__msg__DroneCommand__rosidl_typesupport_introspection_c__get_const_function__DroneCommand__position(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void drone_msgs__msg__DroneCommand__rosidl_typesupport_introspection_c__assign_function__DroneCommand__position(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    drone_msgs__msg__DroneCommand__rosidl_typesupport_introspection_c__get_function__DroneCommand__position(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

static rosidl_typesupport_introspection_c__MessageMember drone_msgs__msg__DroneCommand__rosidl_typesupport_introspection_c__DroneCommand_message_member_array[2] = {
  {
    "command",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(drone_msgs__msg__DroneCommand, command),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "position",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(drone_msgs__msg__DroneCommand, position),  // bytes offset in struct
    NULL,  // default value
    drone_msgs__msg__DroneCommand__rosidl_typesupport_introspection_c__size_function__DroneCommand__position,  // size() function pointer
    drone_msgs__msg__DroneCommand__rosidl_typesupport_introspection_c__get_const_function__DroneCommand__position,  // get_const(index) function pointer
    drone_msgs__msg__DroneCommand__rosidl_typesupport_introspection_c__get_function__DroneCommand__position,  // get(index) function pointer
    drone_msgs__msg__DroneCommand__rosidl_typesupport_introspection_c__fetch_function__DroneCommand__position,  // fetch(index, &value) function pointer
    drone_msgs__msg__DroneCommand__rosidl_typesupport_introspection_c__assign_function__DroneCommand__position,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers drone_msgs__msg__DroneCommand__rosidl_typesupport_introspection_c__DroneCommand_message_members = {
  "drone_msgs__msg",  // message namespace
  "DroneCommand",  // message name
  2,  // number of fields
  sizeof(drone_msgs__msg__DroneCommand),
  drone_msgs__msg__DroneCommand__rosidl_typesupport_introspection_c__DroneCommand_message_member_array,  // message members
  drone_msgs__msg__DroneCommand__rosidl_typesupport_introspection_c__DroneCommand_init_function,  // function to initialize message memory (memory has to be allocated)
  drone_msgs__msg__DroneCommand__rosidl_typesupport_introspection_c__DroneCommand_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t drone_msgs__msg__DroneCommand__rosidl_typesupport_introspection_c__DroneCommand_message_type_support_handle = {
  0,
  &drone_msgs__msg__DroneCommand__rosidl_typesupport_introspection_c__DroneCommand_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_drone_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, drone_msgs, msg, DroneCommand)() {
  if (!drone_msgs__msg__DroneCommand__rosidl_typesupport_introspection_c__DroneCommand_message_type_support_handle.typesupport_identifier) {
    drone_msgs__msg__DroneCommand__rosidl_typesupport_introspection_c__DroneCommand_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &drone_msgs__msg__DroneCommand__rosidl_typesupport_introspection_c__DroneCommand_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
