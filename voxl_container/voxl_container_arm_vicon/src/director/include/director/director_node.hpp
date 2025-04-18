//standard c++ headers
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include <cmath>

#include "rclcpp/rclcpp.hpp" // to use the common pieces of ros2 system
#include <drone_msgs/msg/drone_command.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>

#define MISSION_WAYPOINT 19
using namespace std::chrono_literals; 
using std::placeholders::_1;

enum class Command {
    ARM,
    DISARM,
    GO,
    KILL,
    LAND,
    SHUTDOWN,
    START,
    START_OFFBOARD,
    START_MISSION,
    STOP,
    WAIT,
    YAW,
    UNKNOWN,
};

enum class Status {
    DISARM,
    MOVE,
    HOLD,
    LANDING,
    KILL,
};

Command stringToEnum(const std::string & str);

class Director : public rclcpp::Node{
  public:

    Director();

  private:

    rclcpp::TimerBase::SharedPtr timer_;
    bool timer_up = false;

    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_; // publish the offboard control connection check message
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_; // publish the command received from the groundstation
    rclcpp::Subscription<drone_msgs::msg::DroneCommand>::SharedPtr vehicle_command_receiver_; // receive commands from the ground station
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_; // send the setpoint to px4
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr local_position_subscriber_; // receive the local position of the vehicle
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_subscriber_; // receive the local position of the vehicle

    float setpoint[3];
    float yaw = 3.14;
    float current_position[3];
    int counter = 0;
    bool ready_to_fly = false;
    bool mission_started = false;
    float mission [MISSION_WAYPOINT][4]={ // coordinates in enu, yaw angle must be in [-3.14,3.14]
        {0, 0, 0.9, 3.14},
        {0, 0, 0.9, 2.15},
        {-1.2, -1.8, 0.9, 2.15},
        {-1.2, -1.8, 0.9, 0},
        {0.6, -1.8, 0.9, 0},
        {0.6, -1.8, 0.9, -1.57},
        {0.6, 1.8, 0.9, -1.57},
        {0.6, 1.8, 0.9, 3.14},
        {-1.2, 1.8, 0.9, 3.14},
        {-1.2, 1.8, 0.9, 1.10},
        {0.6, -1.8, 1.7, 1.10},
        {0.6, -1.8, 1.7, 1.57},
        {-1.2, -1.8, 1.7, 1.57},
        {-1.2, -1.8, 1.7, -2.60},
        {0.6, 1.2, 1.7, -2.60},
        {0.6, 1.2, 1.7, -1.57},
        {-1.2, 1.2, 1.7, -1.57},
        {-1.2, 1.2, 1.7, 2.35},
        {0, 0, 1.7, 2.35},
    };
    /*
    float mission [MISSION_WAYPOINT][4]={
        {0, 0, 1, 3.14},
        {0, 0, 1, 1.57},
        {0, 0, 1, 3.14},
        {0, 0, 1, -1.57},
        {0, 0, 1, 3.14},
        {0, 0, 1, 0.01},
        {0, 0, 1, 3.14},
        {0, 0, 1, -0.01},
        {0, 0, 1, 3.14},
        {0.6, 1.2, 1, 3.14},
        {-0.6, 1.2, 1, 3.14},
        {0.6, 1.2, 1, 3.14},
        {0.6, -1.2, 1, 3.14},
        {0.6, 1.2, 1, 3.14},
        {0.6, 1.2, 1, 0},
        {-0.6, 1.2, 1, 0},
        {0.6, 1.2, 1, 0},
        {0.6, -1.2, 1, 0},
        {0.6, 1.2, 1, 0},
        {0, 0, 1, 0},  
    };
    */
    int mission_idx = 0;
    bool position_reached = false;
    long unsigned int arrive_time = 0;
    
    Status state= Status::DISARM;

    px4_msgs::msg::VehicleOdometry odometry_values;

    void timer_callback();
    void local_position_handler(px4_msgs::msg::VehicleOdometry msg);
    void vehicle_status_handler(px4_msgs::msg::VehicleStatus msg);
    void publish_offboard_control_mode();
    void publish_trajectory_setpoint();
    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0, float param3 = 0.0, float param4 = 0.0);
    void message_handler(const drone_msgs::msg::DroneCommand & msg);

};