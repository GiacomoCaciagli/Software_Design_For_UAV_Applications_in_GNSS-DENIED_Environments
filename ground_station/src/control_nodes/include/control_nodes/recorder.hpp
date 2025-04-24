#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <iostream>
#include <sys/stat.h>

//#include "ViconDataStreamSDK_CPP/DataStreamClient.h"

#include "rclcpp/rclcpp.hpp"

#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/sensor_combined.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>

#include <rosbag2_cpp/writer.hpp>

using std::placeholders::_1;
using namespace std;

class SimpleBagRecorder : public rclcpp::Node
{
  public:

    SimpleBagRecorder();

  private:

  	struct stat sb;

    void uwb_callback(std::shared_ptr<rclcpp::SerializedMessage> msg);
    void vicon_callback(std::shared_ptr<rclcpp::SerializedMessage> msg);
    void odometry_callback(std::shared_ptr<rclcpp::SerializedMessage> msg);
    void sensor_callback(std::shared_ptr<rclcpp::SerializedMessage> msg);
    void setpoint_callback(std::shared_ptr<rclcpp::SerializedMessage> msg);

    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr uwb_subscription_;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr vicon_subscription_;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odometry_subscription_;
    rclcpp::Subscription<px4_msgs::msg::SensorCombined>::SharedPtr sensor_subscription_;
    rclcpp::Subscription<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_subscription_;

    std::unique_ptr<rosbag2_cpp::Writer> uwb_writer_;
    std::unique_ptr<rosbag2_cpp::Writer> vicon_writer_;
    std::unique_ptr<rosbag2_cpp::Writer> odometry_writer_;
    std::unique_ptr<rosbag2_cpp::Writer> sensor_writer_;
    std::unique_ptr<rosbag2_cpp::Writer> trajectory_writer_;

};

// C++ Program to test presence of file/Directory
