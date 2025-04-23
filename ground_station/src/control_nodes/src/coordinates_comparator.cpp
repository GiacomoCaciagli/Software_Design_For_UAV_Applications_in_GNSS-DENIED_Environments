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
#include "../include/control_nodes/coordinates_comparator.hpp"

CoordinatesComparator::CoordinatesComparator() : Node("coordinate_publisher"), count_(0) 
{
  RCLCPP_INFO(this->get_logger(), "Starting");
  
  this->declare_parameter<uint16_t>("port", 51002);
  this->get_parameter<uint16_t>("port", port_);

  float position_variance = 0.001;
  float orientation_variance = 0.001;

  odometry_message_vicon.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;
  odometry_message_vicon.position_variance = {position_variance, position_variance, position_variance};
  odometry_message_vicon.orientation_variance = {orientation_variance,orientation_variance,orientation_variance};
  odometry_message_vicon.velocity_frame = px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_UNKNOWN;
  odometry_message_vicon.velocity = {NAN, NAN, NAN};
  odometry_message_vicon.angular_velocity = {NAN, NAN, NAN};
  odometry_message_vicon.velocity_variance = {NAN, NAN, NAN};

  position_variance = position_variance * 10;
  orientation_variance = orientation_variance * 10;

  odometry_message_uwb.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;
  odometry_message_uwb.position_variance = {position_variance, position_variance, position_variance};
  odometry_message_uwb.orientation_variance = {orientation_variance,orientation_variance,orientation_variance};
  odometry_message_uwb.velocity_frame = px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_UNKNOWN;
  odometry_message_uwb.velocity = {NAN,NAN,NAN};
  odometry_message_uwb.angular_velocity = {NAN,NAN,NAN};
  odometry_message_uwb.velocity_variance = {NAN,NAN,NAN};

  vicon_odometry_ = this->create_publisher<px4_msgs::msg::VehicleOdometry>("/vicon_odometry", 10);
  uwb_odometry_ = this->create_publisher<px4_msgs::msg::VehicleOdometry>("/uwb_odometry", 10);
  vicon_odometry_listener_ = this->create_subscription<mocap4r2_msgs::msg::RigidBodies>("/rigid_bodies",1000,std::bind(&CoordinatesComparator::vicon_callback, this, _1));

  timer_vicon_ = this->create_wall_timer(25ms, std::bind(&CoordinatesComparator::timer_callback_vicon,this));
  timer_uwb_ = this->create_wall_timer(25ms, std::bind(&CoordinatesComparator::timer_callback_uwb,this));
    
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

void CoordinatesComparator::vicon_callback(const mocap4r2_msgs::msg::RigidBodies & msg)
{

  odometry_message_vicon.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;
  odometry_message_vicon.position[0] = msg.rigidbodies[0].pose.position.x;
  odometry_message_vicon.position[1] = -msg.rigidbodies[0].pose.position.y;
  odometry_message_vicon.position[2] = -msg.rigidbodies[0].pose.position.z;

  std::array<double,4> rotation = {msg.rigidbodies[0].pose.orientation.w,msg.rigidbodies[0].pose.orientation.x,msg.rigidbodies[0].pose.orientation.y*(-1),msg.rigidbodies[0].pose.orientation.z*(-1)}; // quaternion fixed frame to mobile frame (vicon)
  odometry_message_vicon.q[0] = rotation[0];
  odometry_message_vicon.q[1] = rotation[1];
  odometry_message_vicon.q[2] = rotation[2];
  odometry_message_vicon.q[3] = rotation[3];

  odometry_message_vicon.timestamp = (msg.header.stamp.sec*1000000) + (msg.header.stamp.nanosec / 1000); // VehicleOdometry wants microsecond
}

void CoordinatesComparator::timer_callback_vicon(){
  vicon_odometry_->publish(odometry_message_vicon);
}

void CoordinatesComparator::timer_callback_uwb(){
      
  ssize_t rx_length = recvfrom(sock_, (char *) buffer_, sizeof(buffer_), MSG_WAITALL, (struct sockaddr *) &server_, &socket_length_);
  buffer_[rx_length] = '\0';

  // DEBUG
  RCLCPP_INFO(this->get_logger(), "r<x_length: %ld", rx_length);

  // Unpack from locqt Server
  memcpy(&frame_number_, buffer_+0, 4);
  memcpy(&frame_ID_, buffer_+4, 1);

  memcpy(&translation[0], buffer_+5, 8);
  odometry_message_uwb.position[0] = translation[0];
  memcpy(&translation[1], buffer_+13, 8);
  odometry_message_uwb.position[1] = translation[1];
  memcpy(&translation[2], buffer_+21, 8);
  odometry_message_uwb.position[2] = translation[2];

  memcpy(&rotation_q[0], buffer_+29, 8);
  odometry_message_uwb.q[1] = rotation_q[0];
  memcpy(&rotation_q[1], buffer_+37, 8);
  odometry_message_uwb.q[2] = rotation_q[0];
  memcpy(&rotation_q[2], buffer_+45, 8);
  odometry_message_uwb.q[3] = rotation_q[0];
  memcpy(&rotation_q[3], buffer_+53, 8);
  odometry_message_uwb.q[0] = rotation_q[0];

  uwb_odometry_->publish(odometry_message_uwb);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv); //initialize ros2
  auto node = std::make_shared<CoordinatesComparator>();
  
  node->connect();
  
  rclcpp::spin(node); //start processing data from the node
  
  node->disconnect();
  
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