//standard c++ headers
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include <fstream>

#include <cmath>

#include "rclcpp/rclcpp.hpp" // to use the common pieces of ros2 system
#include "std_msgs/msg/string.hpp" // includes the build-in message type used in this example
#include <drone_msgs/msg/drone_command.hpp>

using namespace std::chrono_literals; 
using std::placeholders::_1;

class CommandPublisher : public rclcpp::Node
{
  public:
    CommandPublisher();

  private:
   	rclcpp::Publisher<drone_msgs::msg::DroneCommand>::SharedPtr command_publisher_;
    std::string commands[13] = {"arm","disarm","go","k","kill","land","mission","offboard","start","stop","w","wait","yaw"};

    std::string command;
};