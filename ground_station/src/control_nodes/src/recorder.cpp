
#include "../include/control_nodes/recorder.hpp"

SimpleBagRecorder::SimpleBagRecorder() : Node("simple_bag_recorder")
{
  uwb_writer_ = std::make_unique<rosbag2_cpp::Writer>();
  vicon_writer_ = std::make_unique<rosbag2_cpp::Writer>();
  odometry_writer_ = std::make_unique<rosbag2_cpp::Writer>();
  sensor_writer_ = std::make_unique<rosbag2_cpp::Writer>();
  trajectory_writer_ = std::make_unique<rosbag2_cpp::Writer>();

  rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
  auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

  int i = 0;
  std::string mission_name = "uwb_fast_mission";
  std::string str_uwb_dir = "bags/input_data_" + mission_name;
  std::string str_vicon_dir = "bags/vicon_data_" + mission_name;
  std::string str_odometry_dir = "bags/output_data_" + mission_name;
  std::string str_sensor_dir = "bags/sensor_data_" + mission_name;
  std::string str_setpoint_dir = "bags/setpoint_" + mission_name;

  std::string uwb_dir = str_uwb_dir + to_string(i);
  std::string vicon_dir = str_vicon_dir + to_string(i);
  std::string odometry_dir = str_odometry_dir + to_string(i);
  std::string sensor_dir = str_sensor_dir + to_string(i);
  std::string setpoint_dir = str_setpoint_dir + to_string(i);

  while (stat(uwb_dir.c_str(), &sb)==0)
  {
    i++;
    uwb_dir = str_uwb_dir + to_string(i);
  }
  i=0;
  while (stat(vicon_dir.c_str(), &sb)==0)
  {
    i++;
    vicon_dir = str_vicon_dir+ to_string(i);
  }
  i=0;
  while (stat(odometry_dir.c_str(), &sb)==0)
  {
    i++;
    odometry_dir = str_odometry_dir + to_string(i);
  }
  i=0;
  while (stat(sensor_dir.c_str(), &sb)==0)
  {
    i++;
    sensor_dir = str_sensor_dir + to_string(i);
  }
  i=0;
  while (stat(setpoint_dir.c_str(), &sb)==0)
  {
    i++;
    setpoint_dir = str_setpoint_dir + to_string(i);
  }

  uwb_writer_->open(uwb_dir);
  vicon_writer_->open(vicon_dir);
  odometry_writer_->open(odometry_dir);
  //sensor_writer_->open(sensor_dir);
  trajectory_writer_->open(setpoint_dir);

  uwb_subscription_ = create_subscription<px4_msgs::msg::VehicleOdometry>("/fmu/in/vehicle_visual_odometry", qos, std::bind(&SimpleBagRecorder::uwb_callback, this, _1));
  vicon_subscription_ = create_subscription<px4_msgs::msg::VehicleOdometry>("/vicon_odometry", 10, std::bind(&SimpleBagRecorder::vicon_callback, this, _1));
  odometry_subscription_ = create_subscription<px4_msgs::msg::VehicleOdometry>("/fmu/out/vehicle_odometry", qos, std::bind(&SimpleBagRecorder::odometry_callback, this, _1));
  //sensor_subscription_ = create_subscription<px4_msgs::msg::SensorCombined>("/fmu/out/sensor_combined", qos, std::bind(&SimpleBagRecorder::sensor_callback, this, _1));
  trajectory_subscription_ = create_subscription<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", qos, std::bind(&SimpleBagRecorder::setpoint_callback, this, _1));
}

void SimpleBagRecorder::uwb_callback(std::shared_ptr<rclcpp::SerializedMessage> msg) 
{
  rclcpp::Time time_stamp = this->now();

  uwb_writer_->write(msg, "/fmu/in/vehicle_visual_odometry", "px4_msgs/msg/VehicleOdometry", time_stamp);
}

void SimpleBagRecorder::vicon_callback(std::shared_ptr<rclcpp::SerializedMessage> msg) 
{
  rclcpp::Time time_stamp = this->now();

  vicon_writer_->write(msg, "/vicon_odometry", "px4_msgs/msg/VehicleOdometry", time_stamp);
}

void SimpleBagRecorder::odometry_callback(std::shared_ptr<rclcpp::SerializedMessage> msg) 
{
  rclcpp::Time time_stamp = this->now();

  odometry_writer_->write(msg, "/fmu/out/vehicle_odometry", "px4_msgs/msg/VehicleOdometry", time_stamp);
}

void SimpleBagRecorder::sensor_callback(std::shared_ptr<rclcpp::SerializedMessage> msg) 
{
  rclcpp::Time time_stamp = this->now();

  sensor_writer_->write(msg, "/fmu/out/sensor_combined", "px4_msgs/msg/SensorCombined", time_stamp);
}

void SimpleBagRecorder::setpoint_callback(std::shared_ptr<rclcpp::SerializedMessage> msg) 
{
  rclcpp::Time time_stamp = this->now();

  trajectory_writer_->write(msg, "/fmu/in/trajectory_setpoint", "px4_msgs/msg/TrajectorySetpoint", time_stamp);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimpleBagRecorder>());
  rclcpp::shutdown();
  return 0;
}