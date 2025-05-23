// Copyright 2022 Intelligent Robotics Lab
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


#include "mocap4r2_vicon_driver/mocap4r2_vicon_driver.hpp"

using namespace std::chrono_literals;

namespace mocap4r2_vicon_driver
{

// The vicon driver node has differents parameters
// to initialized with the mocap4r2_vicon_driver_params.yaml
ViconDriverNode::ViconDriverNode()
: ControlledLifecycleNode("mocap4r2_vicon_driver_node")
{
  declare_parameter<std::string>("stream_mode", "ClientPull");
  declare_parameter<std::string>("host_name", "192.168.10.1:801");
  declare_parameter<std::string>("frame_id", "vicon_world");
}

// In charge of choose the different driver options related and provided by the Vicon SDK
void ViconDriverNode::set_settings_vicon()
{
  ViconDataStreamSDK::CPP::Result::Enum result(ViconDataStreamSDK::CPP::Result::Unknown);
  if (stream_mode_ == "ServerPush") {
    result = client.SetStreamMode(ViconDataStreamSDK::CPP::StreamMode::ServerPush).Result;
  } else if (stream_mode_ == "ClientPull") {
    result = client.SetStreamMode(ViconDataStreamSDK::CPP::StreamMode::ClientPull).Result;
  } else {
    RCLCPP_FATAL(get_logger(), "Unknown stream mode -- options are ServerPush, ClientPull");
    rclcpp::shutdown();
  }

  RCLCPP_INFO(
    get_logger(), "Setting Stream Mode to %s : %s",
    stream_mode_.c_str(), Enum2String(result).c_str());

  client.SetAxisMapping(
    ViconDataStreamSDK::CPP::Direction::Forward,
    ViconDataStreamSDK::CPP::Direction::Left, ViconDataStreamSDK::CPP::Direction::Up);
  ViconDataStreamSDK::CPP::Output_GetAxisMapping _Output_GetAxisMapping = client.GetAxisMapping();

  RCLCPP_INFO(
    get_logger(),
    "Axis Mapping: X-%s Y-%s Z-%s",
    Enum2String(_Output_GetAxisMapping.XAxis).c_str(),
    Enum2String(_Output_GetAxisMapping.YAxis).c_str(),
    Enum2String(_Output_GetAxisMapping.ZAxis).c_str());

  client.EnableSegmentData();
  client.EnableMarkerData();
  client.EnableUnlabeledMarkerData();

  RCLCPP_INFO(
    get_logger(), "IsSegmentDataEnabled? %s",
    client.IsSegmentDataEnabled().Enabled ? "true" : "false");

  ViconDataStreamSDK::CPP::Output_GetVersion _Output_GetVersion = client.GetVersion();

  RCLCPP_INFO(
    get_logger(), "Version: %d.%d.%d",
    _Output_GetVersion.Major,
    _Output_GetVersion.Minor,
    _Output_GetVersion.Point
  );
}

// In charge of the transition of the lifecycle node
void ViconDriverNode::control_start(const mocap4r2_control_msgs::msg::Control::SharedPtr msg)
{
  (void)msg;
}

// In charge of the transition of the lifecycle node
void ViconDriverNode::control_stop(const mocap4r2_control_msgs::msg::Control::SharedPtr msg)
{
  (void)msg;
}

// In charge of get the Vicon information and convert it to vicon_msgs
void ViconDriverNode::process_frame()
{
  if (local_pos_pub_->get_subscription_count() == 0)
  {
    return;
  }

  ViconDataStreamSDK::CPP::Output_GetFrame ans = client.GetFrame();

  if (ans.Result == ViconDataStreamSDK::CPP::Result::Success) {
    ViconDataStreamSDK::CPP::Output_GetFrameNumber OutputFrameNum = client.GetFrameNumber();
    ViconDataStreamSDK::CPP::Output_GetFrameRate OutputFrameRate = client.GetFrameRate();

    RCLCPP_DEBUG(
      get_logger(),
      "GetFrame succeeded. Got frame [%d] at rate [%3.3f]", OutputFrameNum.FrameNumber,
      OutputFrameRate.FrameRateHz);

    // rclcpp::Duration frame_delay = rclcpp::Duration(client.GetLatencyTotal().Total);

    mocap4r2_msgs::msg::RigidBodies rigid_bodies_msg;
    rigid_bodies_msg.header.stamp = now();  // TODO(any): add client.GetLatencyTotal() ?
    rigid_bodies_msg.header.frame_id = frame_id_;
    rigid_bodies_msg.frame_number = frameCount_++;

    auto t = now();

    unsigned int SubjectCount = client.GetSubjectCount().SubjectCount;
    for (unsigned int SubjectIndex = 0; SubjectIndex < SubjectCount; ++SubjectIndex) {
      std::string this_subject_name = client.GetSubjectName(SubjectIndex).SubjectName;

      unsigned int num_subject_segments = client.GetSegmentCount(this_subject_name).SegmentCount;
      for (unsigned int SegmentIndex = 0; SegmentIndex < num_subject_segments; ++SegmentIndex) {
        std::string this_segment_name = client.GetSegmentName(
          this_subject_name, SegmentIndex).SegmentName;

        ViconDataStreamSDK::CPP::Output_GetSegmentGlobalTranslation trans =
          client.GetSegmentGlobalTranslation(this_subject_name, this_segment_name);
        ViconDataStreamSDK::CPP::Output_GetSegmentGlobalRotationQuaternion rot =
          client.GetSegmentGlobalRotationQuaternion(this_subject_name, this_segment_name);

        mocap4r2_msgs::msg::RigidBody this_segment;
        std::string rigid_body_name = this_subject_name + "." + this_segment_name;
        this_segment.rigid_body_name = rigid_body_name;
        rigid_bodies_msg.rigidbodies.push_back(this_segment);

        odometry_values.position[0] = trans.Translation[0] / 1000.0;
        odometry_values.position[1] = trans.Translation[1] / 1000.0 * (-1.0);
        odometry_values.position[2] = trans.Translation[2] / 1000.0 * (-1.0);

        odometry_values.q[0] = rot.Rotation[3];
        odometry_values.q[1] = rot.Rotation[0];
        odometry_values.q[2] = rot.Rotation[1]*(-1.0);
        odometry_values.q[3] = rot.Rotation[2]*(-1.0);

        odometry_values.timestamp = t.nanoseconds() / 1000; // VehicleOdometry wants microsecond

        if (old_time != 0)
        {
          float time_delay = (float)(odometry_values.timestamp - old_time)/1000000;
          odometry_values.velocity[0] = (odometry_values.position[0]-old_position[0])/time_delay;
          odometry_values.velocity[1] = (odometry_values.position[1]-old_position[1])/time_delay;
          odometry_values.velocity[2] = (odometry_values.position[2]-old_position[2])/time_delay;
        }else
        {
          odometry_values.velocity[0] = 0.0;
          odometry_values.velocity[1] = 0.0;
          odometry_values.velocity[2] = 0.0;
        }

        old_position[0] = odometry_values.position[0];
        old_position[1] = odometry_values.position[1];
        old_position[2] = odometry_values.position[2];
        old_time = odometry_values.timestamp;

      }
    }
    // Publishing messages
    if (rigid_bodies_msg.rigidbodies.size() > 0) {
      local_pos_pub_->publish(odometry_values);
    }
  }
}

using CallbackReturnT =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;


// The next Callbacks are used to manage behavior in the different states of the lifecycle node.
CallbackReturnT
ViconDriverNode::on_configure(const rclcpp_lifecycle::State &)
{
  initParameters();

  local_pos_pub_ = create_publisher<px4_msgs::msg::VehicleOdometry>("/fmu/in/vehicle_visual_odometry", 10);

  float position_variance = 0.001;
  float orientation_variance = 0.001;
  float velocity_variance = 0.001;

  odometry_values.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;
  odometry_values.position_variance = {position_variance, position_variance, position_variance};
  odometry_values.orientation_variance = {orientation_variance,orientation_variance,orientation_variance};
  odometry_values.velocity_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;
  odometry_values.angular_velocity = {NAN, NAN, NAN};
  odometry_values.velocity_variance = {velocity_variance, velocity_variance, velocity_variance};

  auto stat = client.Connect(host_name_).Result;

  if (stat == ViconDataStreamSDK::CPP::Result::Success && client.IsConnected().Connected) {
    RCLCPP_INFO(get_logger(), "... connected!");
    return CallbackReturnT::SUCCESS;
  } else {
    RCLCPP_ERROR(get_logger(), "... not connected :(  [%s]", Enum2String(stat).c_str());
    return CallbackReturnT::FAILURE;
  }
}

CallbackReturnT
ViconDriverNode::on_activate(const rclcpp_lifecycle::State &)
{
  local_pos_pub_->on_activate();

  set_settings_vicon();

  // TODO(any): timer freq to be configurable
  timer_ = create_wall_timer(20ms, std::bind(&ViconDriverNode::process_frame, this));

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
ViconDriverNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  local_pos_pub_->on_deactivate();

  client.DisableSegmentData();
  client.DisableMarkerData();
  client.DisableUnlabeledMarkerData();

  timer_ = nullptr;

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
ViconDriverNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  client.Disconnect();
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
ViconDriverNode::on_shutdown(const rclcpp_lifecycle::State &)
{
  client.Disconnect();
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
ViconDriverNode::on_error(const rclcpp_lifecycle::State &)
{
  client.Disconnect();
  return CallbackReturnT::SUCCESS;
}


// Init the necessary parameters to use the Vicon SDK.
void ViconDriverNode::initParameters()
{
  get_parameter<std::string>("stream_mode", stream_mode_);
  get_parameter<std::string>("host_name", host_name_);
  get_parameter<std::string>("frame_id", frame_id_);
}

// Transform the Vicon SDK enumerations to strings
std::string ViconDriverNode::Enum2String(const ViconDataStreamSDK::CPP::Direction::Enum i_Direction)
{
  switch (i_Direction) {
    case ViconDataStreamSDK::CPP::Direction::Forward:
      return "Forward";
    case ViconDataStreamSDK::CPP::Direction::Backward:
      return "Backward";
    case ViconDataStreamSDK::CPP::Direction::Left:
      return "Left";
    case ViconDataStreamSDK::CPP::Direction::Right:
      return "Right";
    case ViconDataStreamSDK::CPP::Direction::Up:
      return "Up";
    case ViconDataStreamSDK::CPP::Direction::Down:
      return "Down";
    default:
      return "Unknown";
  }
}

// Transform the Vicon SDK enumerations to strings
std::string ViconDriverNode::Enum2String(const ViconDataStreamSDK::CPP::Result::Enum i_result)
{
  switch (i_result) {
    case ViconDataStreamSDK::CPP::Result::ClientAlreadyConnected:
      return "ClientAlreadyConnected";
    case ViconDataStreamSDK::CPP::Result::ClientConnectionFailed:
      return "ClientConnectionFailed";
    case ViconDataStreamSDK::CPP::Result::CoLinearAxes:
      return "CoLinearAxes";
    case ViconDataStreamSDK::CPP::Result::InvalidDeviceName:
      return "InvalidDeviceName";
    case ViconDataStreamSDK::CPP::Result::InvalidDeviceOutputName:
      return "InvalidDeviceOutputName";
    case ViconDataStreamSDK::CPP::Result::InvalidHostName:
      return "InvalidHostName";
    case ViconDataStreamSDK::CPP::Result::InvalidIndex:
      return "InvalidIndex";
    case ViconDataStreamSDK::CPP::Result::InvalidLatencySampleName:
      return "InvalidLatencySampleName";
    case ViconDataStreamSDK::CPP::Result::InvalidMarkerName:
      return "InvalidMarkerName";
    case ViconDataStreamSDK::CPP::Result::InvalidMulticastIP:
      return "InvalidMulticastIP";
    case ViconDataStreamSDK::CPP::Result::InvalidSegmentName:
      return "InvalidSegmentName";
    case ViconDataStreamSDK::CPP::Result::InvalidSubjectName:
      return "InvalidSubjectName";
    case ViconDataStreamSDK::CPP::Result::LeftHandedAxes:
      return "LeftHandedAxes";
    case ViconDataStreamSDK::CPP::Result::NoFrame:
      return "NoFrame";
    case ViconDataStreamSDK::CPP::Result::NotConnected:
      return "NotConnected";
    case ViconDataStreamSDK::CPP::Result::NotImplemented:
      return "NotImplemented";
    case ViconDataStreamSDK::CPP::Result::ServerAlreadyTransmittingMulticast:
      return "ServerAlreadyTransmittingMulticast";
    case ViconDataStreamSDK::CPP::Result::ServerNotTransmittingMulticast:
      return "ServerNotTransmittingMulticast";
    case ViconDataStreamSDK::CPP::Result::Success:
      return "Success";
    case ViconDataStreamSDK::CPP::Result::Unknown:
      return "Unknown";
    default:
      return "unknown";
  }
}

}  // namespace mocap4r2_vicon_driver
