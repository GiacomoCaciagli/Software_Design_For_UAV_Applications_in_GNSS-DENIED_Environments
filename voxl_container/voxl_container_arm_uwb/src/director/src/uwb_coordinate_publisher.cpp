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

//standard c++ headers
#include "../include/director/uwb_coordinate_publisher.hpp"

CoordinatesComparator::CoordinatesComparator() : Node("coordinate_publisher"), count_(0) 
{
  RCLCPP_INFO(this->get_logger(), "Starting");
  //data_file.open ("data.txt");

  this->declare_parameter<uint16_t>("port", 51002);
  this->get_parameter<uint16_t>("port", port_);

  float position_variance = 0.25;
  float orientation_variance = 0.001;

  odometry_message_uwb.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;
  odometry_message_uwb.position_variance = {position_variance, position_variance, position_variance/300};
  odometry_message_uwb.orientation_variance = {orientation_variance,orientation_variance,orientation_variance};
  odometry_message_uwb.velocity_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;
  odometry_message_uwb.velocity = {NAN,NAN, 0.0};
  odometry_message_uwb.angular_velocity = {NAN,NAN,NAN};
  odometry_message_uwb.velocity_variance = {NAN,NAN,orientation_variance * 2};

  uwb_odometry_ = this->create_publisher<px4_msgs::msg::VehicleOdometry>("/fmu/in/vehicle_visual_odometry", 10);
  timer_uwb_ = this->create_wall_timer(10ms, std::bind(&CoordinatesComparator::timer_callback_uwb,this));
    
} 

void CoordinatesComparator::connect()
{
  // Signal Handler
  signal(SIGINT, [](int sig_num){exit(sig_num);});
  
  // Socket init
  if( (sock_ = socket(AF_INET, SOCK_DGRAM, 0)) < 0) 
  {
      RCLCPP_ERROR(this->get_logger(), "Socket creation failed"); 
      exit(EXIT_FAILURE);
  }
  else
  {
      RCLCPP_INFO(this->get_logger(),"Socket created");
  }

  // Zero-out server address structure
  std::memset(&server_, 0, sizeof(server_));

  server_.sin_addr.s_addr = INADDR_ANY; //inet_addr("192.168.50.56");
  server_.sin_family = AF_INET;
  server_.sin_port = htons(port_);
  socket_length_ = sizeof(server_);

  // Bind the socket to any valid IP address and a specific port
  RCLCPP_INFO(this->get_logger(), "Waiting for bind...");
  if( bind(sock_, (const struct sockaddr *)&server_, sizeof(server_)) < 0 ) 
  { 
      RCLCPP_ERROR(this->get_logger(),"Bind failed");
      sleep(1);
      // exit(EXIT_FAILURE); 
  }
  else 
  {
      RCLCPP_INFO(this->get_logger(), "Bind completed to ADDR: %d, PORT: %u", ntohl(server_.sin_addr.s_addr), ntohs(server_.sin_port));
  }
}

void CoordinatesComparator::disconnect()
{
  close(sock_);
}

void CoordinatesComparator::close_file()
{
  //data_file.close();
}

void CoordinatesComparator::timer_callback_uwb(){
      
  ssize_t rx_length = recvfrom(sock_, (char *) buffer_, sizeof(buffer_), MSG_WAITALL, (struct sockaddr *) &server_, &socket_length_);
  buffer_[rx_length] = '\0';

  auto start = now();
  // DEBUG
  // RCLCPP_INFO(this->get_logger(), "r<x_length: %ld", rx_length);

  // Unpack from locqt Server
  memcpy(&frame_number_, buffer_+0, 4);
  memcpy(&frame_ID_, buffer_+4, 1);

  memcpy(&translation[0], buffer_+5, 8);
  odometry_message_uwb.position[0] = translation[0];
  memcpy(&translation[1], buffer_+13, 8);
  odometry_message_uwb.position[1] = translation[1]*(-1.0);
  memcpy(&translation[2], buffer_+21, 8);
  odometry_message_uwb.position[2] = translation[2]*(-1.0);

  memcpy(&rotation_q[0], buffer_+29, 8);
  odometry_message_uwb.q[1] = rotation_q[0];
  memcpy(&rotation_q[1], buffer_+37, 8);
  odometry_message_uwb.q[2] = rotation_q[1]*(-1.0);
  memcpy(&rotation_q[2], buffer_+45, 8);
  odometry_message_uwb.q[3] = rotation_q[2]*(-1.0);
  memcpy(&rotation_q[3], buffer_+53, 8);
  odometry_message_uwb.q[0] = rotation_q[3];

  auto t = now();

  odometry_message_uwb.timestamp = (t.nanoseconds() / 1000); // VehicleOdometry wants microsecond

  
  if (old_time != 0)
  {
    time_delay = (float)(odometry_message_uwb.timestamp - old_time)/1000000;
    //odometry_message_uwb.velocity[0] = (odometry_message_uwb.position[0]-old_position[0])/time_delay;
    //odometry_message_uwb.velocity[1] = (odometry_message_uwb.position[1]-old_position[1])/time_delay;
    odometry_message_uwb.velocity[2] = (odometry_message_uwb.position[2]-old_position[2])/time_delay;
  }/*else
  {
    odometry_message_uwb.velocity[0] = 0.0;
    odometry_message_uwb.velocity[1] = 0.0;
    odometry_message_uwb.velocity[2] = 0.0;
    start_time = odometry_message_uwb.timestamp;
  }*/

  
  old_position[0] = odometry_message_uwb.position[0];
  old_position[1] = odometry_message_uwb.position[1];
  old_position[2] = odometry_message_uwb.position[2];
  old_time = odometry_message_uwb.timestamp;
  

  uwb_odometry_->publish(odometry_message_uwb);
  /*
  float relative_time = (float)(odometry_message_uwb.timestamp - start_time)/1000000;
  data_file << "Timestamp: " << odometry_message_uwb.timestamp << "\n";
  data_file << "Timestamp relative: " << relative_time << "\n";
  data_file << "Time delay: " << time_delay << "\n";
  data_file << "Position: x: " << odometry_message_uwb.position[0] << "\ty: " << odometry_message_uwb.position[1] << "\tz: " << odometry_message_uwb.position[2] << "\n";
  data_file << "Velocity: vx: " << odometry_message_uwb.velocity[0] << "\tvy: " << odometry_message_uwb.velocity[1] << "\tvz: " << odometry_message_uwb.velocity[2] << "\n";
  data_file << "Orientation: w: " << odometry_message_uwb.q[0] << "\tqx: " << odometry_message_uwb.q[1] << "\tqy: " << odometry_message_uwb.q[2] << "\tqz: " << odometry_message_uwb.q[3] << "\n\n";
  data_file.flush();
  */
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv); //initialize ros2
  auto node = std::make_shared<CoordinatesComparator>();
  
  node->connect();
  
  rclcpp::spin(node); //start processing data from the node
  
  node->disconnect();
  //node->close_file();
  
  rclcpp::shutdown();
  
  return 0;
}

/**
 * Payload Data Structure from locqt Server (bytes)
 * 0-3      ->      frame_number
 * 4        ->      frame_ID
 * 5-12     ->      translation.x
 * 13-20    ->      translation.y   
 * 21-28    ->      translation.z
 * 29-36    ->      rotation.quaternion.x
 * 37-44    ->      rotation.quaternion.y
 * 45-52    ->      rotation.quaternion.z
 * 53-60    ->      rotation.quaternion.scalar
 */