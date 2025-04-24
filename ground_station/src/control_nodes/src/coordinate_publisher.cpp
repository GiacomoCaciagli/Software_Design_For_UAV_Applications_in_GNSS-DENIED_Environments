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
#include "../include/control_nodes/coordinate_publisher.hpp"

CoordinatePublisher::CoordinatePublisher() : Node("coordinate_publisher"), count_(0) 
{
  RCLCPP_INFO(this->get_logger(), "Starting");
  data_file.open ("data1.txt");
  vehicle_local_position_publisher_ = this->create_publisher<px4_msgs::msg::VehicleOdometry>("/vicon_odometry", 10);
  vicon_odometry_listener_ = this->create_subscription<mocap4r2_msgs::msg::RigidBodies>("/rigid_bodies",1000,std::bind(&CoordinatePublisher::topic_callback, this, _1));
  
  float position_variance = 0.001;
  float orientation_variance = 0.001;

  odometry_values.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;
  odometry_values.position_variance = {position_variance, position_variance, position_variance};
  odometry_values.orientation_variance = {orientation_variance,orientation_variance,orientation_variance};
  odometry_values.velocity_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;
  odometry_values.angular_velocity = {NAN, NAN, NAN};
  odometry_values.velocity_variance = {0.001, 0.001, 0.001};

  timer_ = this->create_wall_timer(25ms, std::bind(&CoordinatePublisher::timer_callback,this));

} 

void CoordinatePublisher::topic_callback(const mocap4r2_msgs::msg::RigidBodies & msg)
{

  odometry_values.position[0] = msg.rigidbodies[0].pose.position.x;
  odometry_values.position[1] = msg.rigidbodies[0].pose.position.y*(-1.0);
  odometry_values.position[2] = msg.rigidbodies[0].pose.position.z*(-1.0);

  std::array<double,4> rotation = {msg.rigidbodies[0].pose.orientation.w,msg.rigidbodies[0].pose.orientation.x,msg.rigidbodies[0].pose.orientation.y*(-1.0),msg.rigidbodies[0].pose.orientation.z*(-1.0)}; // quaternion fixed frame to mobile frame (vicon)
  odometry_values.q[0] = rotation[0];
  odometry_values.q[1] = rotation[1];
  odometry_values.q[2] = rotation[2];
  odometry_values.q[3] = rotation[3];
  
  auto t = now();

  odometry_values.timestamp = (t.nanoseconds() / 1000); // VehicleOdometry wants microsecond

  if (old_time != 0)
  {
    time_delay = (float)(odometry_values.timestamp - old_time)/1000000;
    odometry_values.velocity[0] = (odometry_values.position[0]-old_position[0])/time_delay;
    odometry_values.velocity[1] = (odometry_values.position[1]-old_position[1])/time_delay;
    odometry_values.velocity[2] = (odometry_values.position[2]-old_position[2])/time_delay;
  }else
  {
    odometry_values.velocity[0] = 0.0;
    odometry_values.velocity[1] = 0.0;
    odometry_values.velocity[2] = 0.0;
    start_time = odometry_values.timestamp;
  }

  old_position[0] = odometry_values.position[0];
  old_position[1] = odometry_values.position[1];
  old_position[2] = odometry_values.position[2];
  old_time = odometry_values.timestamp;

}

void CoordinatePublisher::timer_callback()
{      
  if (old_timestamp != odometry_values.timestamp)
  {
    old_timestamp = odometry_values.timestamp;
    vehicle_local_position_publisher_->publish(odometry_values);
    float relative_time = (float)(odometry_values.timestamp - start_time)/1000000;
    data_file << "Timestamp: " << odometry_values.timestamp << "\n";
    data_file << "Timestamp relative: " << relative_time << "\n";
    data_file << "Time delay: " << time_delay << "\n";
    data_file << "Position: x: " << odometry_values.position[0] << "\ty: " << odometry_values.position[1] << "\tz: " << odometry_values.position[2] << "\n";
    data_file << "Velocity: vx: " << odometry_values.velocity[0] << "\tvy: " << odometry_values.velocity[1] << "\tvz: " << odometry_values.velocity[2] << "\n";
    data_file << "Orientation: w: " << odometry_values.q[0] << "\tqx: " << odometry_values.q[1] << "\tqy: " << odometry_values.q[2] << "\tqz: " << odometry_values.q[3] << "\n\n";
    data_file.flush();
  }
}

void CoordinatePublisher::close()
{
  data_file.close();
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv); //initialize ros2
  auto node = std::make_shared<CoordinatePublisher>();
    
  rclcpp::spin(node); //start processing data from the node
  
  node->close();

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