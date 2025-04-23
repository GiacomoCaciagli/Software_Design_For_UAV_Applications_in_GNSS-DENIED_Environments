#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include <fstream>

#include <csignal>
#include <bits/stdc++.h> 
#include <stdlib.h> 
#include <unistd.h> 
#include <sys/types.h> 
#include <sys/socket.h> 
#include <arpa/inet.h> 
#include <netinet/in.h> 
#include <cstdlib>
#include <cmath>

#include <typeinfo>

#include "rclcpp/rclcpp.hpp" // to use the common pieces of ros2 system
#include "std_msgs/msg/string.hpp" // includes the build-in message type used in this example
#include <mocap4r2_msgs/msg/rigid_bodies.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>

#define MAXLINE 1024

using namespace std::chrono_literals; 
using std::placeholders::_1;

class CoordinatePublisher : public rclcpp::Node
{
  public:
    CoordinatePublisher();
    void close();

  
  private:

    void topic_callback( const mocap4r2_msgs::msg::RigidBodies & msg);
    void timer_callback();

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<mocap4r2_msgs::msg::RigidBodies>::SharedPtr vicon_odometry_listener_;
   	rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr vehicle_local_position_publisher_;
    
    px4_msgs::msg::VehicleOdometry odometry_values;
    float old_position[3] = {0.0,0.0,0.0};
    long int old_time = 0;
    long int start_time = 0;
    float time_delay = 0;

    size_t count_;
    uint64_t old_timestamp = 0;

    std::ofstream data_file;


};