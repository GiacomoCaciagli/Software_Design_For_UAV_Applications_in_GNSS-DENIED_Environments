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

#include "rclcpp/rclcpp.hpp" // to use the common pieces of ros2 system
#include "std_msgs/msg/string.hpp" // includes the build-in message type used in this example
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
    void close_file();
  
  private:

    void timer_callback_uwb();

    px4_msgs::msg::VehicleOdometry odometry_message_uwb;
    float old_position[3] = {0.0,0.0,0.0};
    long int old_time = 0;
    long int start_time = 0;
    float time_delay = 0;
    //std::ofstream data_file;
    
   	rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr uwb_odometry_;

    rclcpp::TimerBase::SharedPtr timer_uwb_;

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