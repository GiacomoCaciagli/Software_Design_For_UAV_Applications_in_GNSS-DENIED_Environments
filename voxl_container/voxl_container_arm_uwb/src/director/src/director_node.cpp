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

#include "../include/director/director_node.hpp"

/**
 * @brief transforms input string to command
 */
Command stringToEnum(const std::string& str) {
    if (str == "arm") {
        return Command::ARM;
    } else if (str == "disarm") {
        return Command::DISARM;
    } else if (str == "go") {
        return Command::GO;
    } else if (str == "kill" || str == "k") {
        return Command::KILL;
    } else if (str == "land") {
        return Command::LAND;
    } else if (str == "mission"){
        return Command::START_MISSION;
    } else if (str == "offboard") {
        return Command::START_OFFBOARD;
    } else if (str == "start") {
        return Command::START;
    } else if (str == "stop") {
        return Command::STOP;
    } else if (str == "wait" || str == "w") {
        return Command::WAIT;
    } else if (str == "yaw") {
        return Command::YAW;
    } else {
        return Command::UNKNOWN;
    }
}

/**
 * @brief Initializer
 */
Director::Director() : Node("director_node")
{
    //RCLCPP_INFO(this->get_logger(), "Starting");

    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

    offboard_control_mode_publisher_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
    vehicle_command_publisher_ = this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);
    vehicle_command_receiver_ = this->create_subscription<drone_msgs::msg::DroneCommand>("/drone_command",10,std::bind(&Director::message_handler,this,_1));
    trajectory_setpoint_publisher_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
    local_position_subscriber_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>("/fmu/out/vehicle_odometry", qos, std::bind(&Director::local_position_handler,this,_1));
    vehicle_status_subscriber_ = this->create_subscription<px4_msgs::msg::VehicleStatus>("/fmu/out/vehicle_status", qos, std::bind(&Director::vehicle_status_handler,this,_1));

} 

/**
 * @brief Timer used to publish the offboard control message (5 hz) and
 *        to repeat the setpoint (1 hz)
 */
void Director::timer_callback()
{
    // offboard_control_mode needs to be paired with trajectory_setpoint
    publish_offboard_control_mode();

    if (counter < 4)
    {
        counter++;
    }else
    {
        if (state != Status::LANDING && state != Status::DISARM)
        {
            publish_trajectory_setpoint();
        }
        ready_to_fly = true;
    }
}

/**
 * @brief Checks the position of the vehicle, if it reached the destination
 *        pass to hold mode or go to the next waypoint, the position is reached if
 *        the vehicle stays in a range of [+-15 cm] for x and y and [-11,+5 cm] for z
 *          
 *        Disarm the vehicle at an altitude of 15 cm or less during landing
 */
void Director::local_position_handler(px4_msgs::msg::VehicleOdometry msg)
{
    if ((state == Status::MOVE) || (state == Status::HOLD))
    {
        //RCLCPP_INFO(this->get_logger(), "Setpoint:%f,%f,%f ",msg.position[0],msg.position[1],msg.position[2]); 
        current_position[0] = msg.position[0];
        current_position[1] = msg.position[1];
        current_position[2] = msg.position[2];
        bool x_pos = sqrt(pow(msg.position[0]-setpoint[0],2))<0.15; // sphere with 15 cm radius
        bool y_pos = sqrt(pow(msg.position[1]-setpoint[1],2))<0.15;
        bool z_pos = sqrt(pow(msg.position[2]-(setpoint[2]+0.03),2))<0.08;

        //RCLCPP_INFO(this->get_logger(), "xpos:%d, ypos:%d, zpos:%d",x_pos,y_pos,z_pos);

        if (x_pos && y_pos && z_pos)
        {

            bool qw,qz;

            qw = sqrt(pow(msg.q[0]-cos(yaw/2),2))<0.03;
            qz = sqrt(pow(msg.q[3]-sin(yaw/2),2))<0.03;
                
            //RCLCPP_INFO(this->get_logger(), "first qw:%d, qz:%d",qw,qz);
            if (qw && qz)
            {
                position_reached = true;
                state = Status::HOLD;
            }else
            {
                qw = sqrt(pow(msg.q[0]+cos(yaw/2),2))<0.03;
                qz = sqrt(pow(msg.q[3]-sin(-yaw/2),2))<0.03;
                //RCLCPP_INFO(this->get_logger(), "second qw:%d, qz:%d",qw,qz);
                if (qw && qz)
                {
                    state = Status::HOLD;
                    position_reached = true;
                }
                else
                    position_reached = false;
            }
        }else
        {
            position_reached = false;
        }

        if (position_reached && mission_started)
        {
            if (arrive_time == 0)
            {
                arrive_time = msg.timestamp;
                //RCLCPP_INFO(this->get_logger(), "arrive time:%ld",arrive_time);
            }else
            {
                if (msg.timestamp > (arrive_time + 2000000))
                {
                    if (mission_idx == MISSION_WAYPOINT)
                    {
                        setpoint[0] = 0;
                        setpoint[1] = 0;
                        setpoint[2] = 0;
                        state = Status::LANDING;
                        this->publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND, 0.0, 0.0, 0.0, yaw); // pass to land mode
                        mission_started = false;
                    }else
                    {
                        state = Status::HOLD;
                        setpoint[0] = mission[mission_idx][0];
                        setpoint[1] = mission[mission_idx][1]*(-1.0);
                        setpoint[2] = mission[mission_idx][2]*(-1.0);
                        yaw = mission[mission_idx][3];
                        mission_idx++;
                        publish_trajectory_setpoint();
                        state = Status::MOVE;
                    }
                    //RCLCPP_INFO(this->get_logger(), "mission idx:%d",mission_idx);
                }
            }
        }else
        {
            arrive_time = 0;
        }
    }else
    {
        if (state == Status::LANDING)
        {
            current_position[2] = msg.position[2];
            bool z_pos = sqrt(pow(msg.position[2],2))<0.15;

            if (z_pos)
            {
                this->publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0); // disarm the vehicle
                //RCLCPP_INFO(this->get_logger(), "Landing terminated");
            }
        }
    }
}

/**
 * @brief Checks the status of the vehicle
 */
void Director::vehicle_status_handler(px4_msgs::msg::VehicleStatus msg)
{
    if (msg.nav_state != 14)
    {
        setpoint[0] = current_position[0];
        setpoint[1] = current_position[1];
        setpoint[2] = current_position[2];
    }

    if (msg.arming_state == 1)
    {
        state = Status::DISARM;
    }
    /*
    nav_state=14 offboard mode
    nav_state=4  starting state and hold mode
    */
}

/**
 * @brief Publishes the OffboardControlMode message to maintain the Offboard mode
 */
void Director::publish_offboard_control_mode()
{
    px4_msgs::msg::OffboardControlMode msg{};
	msg.position = true;
	msg.velocity = false;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	offboard_control_mode_publisher_->publish(msg);
}

/**
 * @brief publishes the trajectory setpoint
 */
void Director::publish_trajectory_setpoint()
{
    px4_msgs::msg::TrajectorySetpoint msg{};
	msg.position = {setpoint[0], setpoint[1], setpoint[2]};
	msg.yaw = yaw; // [-PI:PI]
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	trajectory_setpoint_publisher_->publish(msg);
}

/**
 * @brief Publishes the vehicle command
*/
void Director::publish_vehicle_command(uint16_t command, float param1, float param2, float param3, float param4)
{
	px4_msgs::msg::VehicleCommand msg{};
	std::cout << std::to_string(command) << std::endl;
	msg.param1 = param1;
	msg.param2 = param2;
    msg.param3 = param3;
    msg.param4 = param4;
	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	vehicle_command_publisher_->publish(msg);
}

/**
 * @brief Handle the message received from the command publisher
 */
void Director::message_handler(const drone_msgs::msg::DroneCommand & msg)
{

    Command value = stringToEnum(msg.command);

    switch (value){
        case Command::ARM:
        {
            //RCLCPP_INFO(this->get_logger(), "ARM command received");
            if (timer_up)
            {
                if (!ready_to_fly)
                {
                    break;
                }
                setpoint[0] = current_position[0];
                setpoint[1] = current_position[1];
                setpoint[2] = current_position[2];
                mission_started = false;
                this->publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0); // arm the vehicle
                state = Status::HOLD;
            }/*else
            {
                //RCLCPP_INFO(this->get_logger(), "Missing offboard signal");
            }*/
            
            break;
        }
        case Command::DISARM: // works only if we are landed
        {
            //RCLCPP_INFO(this->get_logger(), "DISARM command received");
            this->publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0); // disarm the vehicle
            state = Status::DISARM;
            break;
        }
        case Command::GO: // works
        {
            //RCLCPP_INFO(this->get_logger(), "GO command received");
            if ((state == Status::HOLD) || (state == Status::MOVE))
            {
                state = Status::HOLD;
                setpoint[0] = msg.position[0];
                setpoint[1] = msg.position[1]*(-1.0);
                setpoint[2] = msg.position[2]*(-1.0);
                publish_trajectory_setpoint();
                state = Status::MOVE;
            }
            break;
        }
        case Command::KILL: // works
        {
            //RCLCPP_INFO(this->get_logger(), "KILL command received");
            this->publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0, 21196.0);
            counter = 0;
            timer_up = false;
            timer_->cancel();
            state = Status::KILL;
            ready_to_fly = false;
            break;
        }
        case Command::LAND: // 
        {
            //RCLCPP_INFO(this->get_logger(), "LAND command received");
            if (state == Status::HOLD)
            {
                state = Status::LANDING;
                this->publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND, 0.0, 0.0, 0.0, yaw); // pass to land mode, seems working with the dronw
            } 
            break;
        }
        case Command::START:
        {
            //RCLCPP_INFO(this->get_logger(), "START command received");
            if (!timer_up)
            {
                counter = 0;

                timer_ = this->create_wall_timer(250ms, std::bind(&Director::timer_callback,this));
                timer_up = true;
            }
            break;
        }
        case Command::START_OFFBOARD:
        {
            //RCLCPP_INFO(this->get_logger(), "OFFBOARD command received");
            if (timer_up)
            {
                if (counter<4)
                {
                    break;
                }
                this->publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6); // pass to offboard mode
                state = Status::DISARM;
            }/*else
            {
                RCLCPP_INFO(this->get_logger(), "Missing offboard signal");
            }*/
            break;
        }
        case Command::START_MISSION:
        {
            //RCLCPP_INFO(this->get_logger(), "MISSION command received");
            if (timer_up && (state == Status::HOLD))
            {
                mission_idx = 0;
                setpoint[0] = mission[mission_idx][0];
                setpoint[1] = mission[mission_idx][1]*(-1.0);
                setpoint[2] = mission[mission_idx][2]*(-1.0);
                yaw = mission[mission_idx][3];
                mission_idx++;
                publish_trajectory_setpoint();
                state = Status::MOVE;
                mission_started = true;
                //RCLCPP_INFO(this->get_logger(), "%f,%f,%f,%f", setpoint[0],setpoint[1],setpoint[2],yaw);
            }/*else
            {
                RCLCPP_INFO(this->get_logger(), "MISSION ERROR");
                std::cout << timer_up;
            }*/

            break;
        }
        case Command::STOP:
        {
            //RCLCPP_INFO(this->get_logger(), "STOP command received");
            counter = 0;
            timer_up = false;
            if (abs(current_position[2])>0.1)
            {
                state = Status::LANDING;
                this->publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND, 0.0, 0.0, 0.0, yaw); // pass to land mode
            }
            timer_->cancel();
            ready_to_fly = false;
            break;
        }   
        case Command::WAIT:
        { 
            //RCLCPP_INFO(this->get_logger(), "WAIT command received");
            if (state ==Status::MOVE)
            {
                mission_started = false;
                setpoint[0] = current_position[0];
                setpoint[1] = current_position[1];
                setpoint[2] = current_position[2];
            }
            break;
        }
        case Command::YAW:
        {
            RCLCPP_INFO(this->get_logger(), "YAW command received");
            if ((state == Status::HOLD))
            {
                state = Status::HOLD;
                yaw = msg.position[0];
                publish_trajectory_setpoint();    
                state = Status::MOVE;
            }
            break;
        }
        default:
        {
            break;
        }    
    }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv); //initialize ros2
  rclcpp::spin(std::make_shared<Director>()); //start processing data from the node
  rclcpp::shutdown();
  return 0;
}
