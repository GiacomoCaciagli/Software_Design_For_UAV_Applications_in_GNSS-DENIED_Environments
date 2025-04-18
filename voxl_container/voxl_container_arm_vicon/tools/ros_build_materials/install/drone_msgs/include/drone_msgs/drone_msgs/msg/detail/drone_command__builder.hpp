// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from drone_msgs:msg/DroneCommand.idl
// generated code does not contain a copyright notice

#ifndef DRONE_MSGS__MSG__DETAIL__DRONE_COMMAND__BUILDER_HPP_
#define DRONE_MSGS__MSG__DETAIL__DRONE_COMMAND__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "drone_msgs/msg/detail/drone_command__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace drone_msgs
{

namespace msg
{

namespace builder
{

class Init_DroneCommand_position
{
public:
  explicit Init_DroneCommand_position(::drone_msgs::msg::DroneCommand & msg)
  : msg_(msg)
  {}
  ::drone_msgs::msg::DroneCommand position(::drone_msgs::msg::DroneCommand::_position_type arg)
  {
    msg_.position = std::move(arg);
    return std::move(msg_);
  }

private:
  ::drone_msgs::msg::DroneCommand msg_;
};

class Init_DroneCommand_command
{
public:
  Init_DroneCommand_command()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DroneCommand_position command(::drone_msgs::msg::DroneCommand::_command_type arg)
  {
    msg_.command = std::move(arg);
    return Init_DroneCommand_position(msg_);
  }

private:
  ::drone_msgs::msg::DroneCommand msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::drone_msgs::msg::DroneCommand>()
{
  return drone_msgs::msg::builder::Init_DroneCommand_command();
}

}  // namespace drone_msgs

#endif  // DRONE_MSGS__MSG__DETAIL__DRONE_COMMAND__BUILDER_HPP_
