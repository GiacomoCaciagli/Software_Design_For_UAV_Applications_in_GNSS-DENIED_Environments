// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "../include/control_nodes/command_publisher.hpp"

CommandPublisher::CommandPublisher() : Node("command_publisher")
{
  RCLCPP_INFO(this->get_logger(), "Starting");
  command_publisher_ = this->create_publisher<drone_msgs::msg::DroneCommand>("/drone_command", 10);

  do
  {
    command = "EOF";
    std::cout << "Insert a command:";
    std::cin >> command;
    bool find = false;

    for (auto i : commands)
    {
        if (i==command)
        {
            find = true;
            break;
        }
    }

    if (find)
    {
        if (command=="shutdown")
        {
            command="EOF";
            break;
        }
        drone_msgs::msg::DroneCommand msg;
        msg.command = command;
        if (command=="go")
        {
            std::cout << "Insert the coordinates in ENU frame\n" << std::endl;
            std::cout << "x:";
            std::cin >> msg.position[0];
            std::cout << "y:";
            std::cin >> msg.position[1];
            std::cout << "z:";
            std::cin >> msg.position[2]; 

            printf("Coordinates in NED frame:%.3f,%.3f,%.3f\n",msg.position[0],msg.position[1]*(-1),msg.position[2]*(-1));
        }else
        {
          if (command == "yaw")
          {
            std::cout << "Insert the yaw angle in degrees in NED frame:" << std::endl;
            std::cin >> msg.position[0];
            if (msg.position[0]<0)
              msg.position[0] = msg.position[0] * (-1.0);

            while (msg.position[0] >= 360)
              msg.position[0] = msg.position[0] - 360;
            
            if (msg.position[0] > 180)
            {
              msg.position[0] = -((360 - msg.position[0]) * 3.14159 / 180);
            }else
            {
              msg.position[0] = msg.position[0] * 3.14159 / 180;
            }
            
            printf("yaw angle in radiants:%.3f\n",msg.position[0]);
          }
        }

        command_publisher_->publish(msg);
    }else
    {
        std::cout << "command not found, insert again" << std::endl;
    }

  }
  while (command != "EOF");

  myfile.close();

}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv); //initialize ros2
  rclcpp::spin(std::make_shared<CommandPublisher>()); //start processing data from the node
  rclcpp::shutdown();
  return 0;
}
