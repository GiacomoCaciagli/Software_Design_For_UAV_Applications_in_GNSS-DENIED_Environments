#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>

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

#include "rclcpp/rclcpp.hpp" // to use the common pieces of ros2 system
#include "std_msgs/msg/string.hpp" // includes the build-in message type used in this example
#include <mocap4r2_msgs/msg/rigid_bodies.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>

#define MAXLINE 1024

using namespace std::chrono_literals; 
using std::placeholders::_1;

class CoordinatesComparator : public rclcpp::Node
{
  public:
    CoordinatesComparator();

    void connect();
    void disconnect();
  
  private:

    void vicon_callback( const mocap4r2_msgs::msg::RigidBodies & msg);
    void timer_callback_vicon();
    void timer_callback_uwb();

    rclcpp::TimerBase::SharedPtr timer_vicon_;
    rclcpp::TimerBase::SharedPtr timer_uwb_;

    px4_msgs::msg::VehicleOdometry odometry_message_vicon;
    px4_msgs::msg::VehicleOdometry odometry_message_uwb;
    
    rclcpp::Subscription<mocap4r2_msgs::msg::RigidBodies>::SharedPtr vicon_odometry_listener_;
   	rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr vicon_odometry_;
   	rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr uwb_odometry_;

    uint32_t frame_number_;
    uint8_t frame_ID_;

    int sock_;
    uint16_t port_;
    struct sockaddr_in server_;
    char buffer_[MAXLINE];   
    socklen_t socket_length_; 
    double translation[3] = {0};
    double rotation_q[4] = {0};

    size_t count_;

};