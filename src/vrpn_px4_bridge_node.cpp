// Copyright 2026 Riccardo Enrico
// SPDX-License-Identifier: BSD-3-Clause

/*
 * vrpn_px4_bridge
 *
 * Subscribes to a `geometry_msgs/PoseStamped` produced by `vrpn_mocap`
 * (Vicon / OptiTrack over VRPN) and republishes it as a
 * `px4_msgs/VehicleOdometry` on `/fmu/in/vehicle_visual_odometry`, converting
 * the mocap ENU world frame to PX4 NED (see frame_transforms.hpp).
 *
 * C++ for low, deterministic latency in the tracking callback path.
 */

#include <limits>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"

#include "vrpn_px4_bridge/frame_transforms.hpp"

namespace vrpn_px4_bridge
{

class BridgeNode : public rclcpp::Node
{
public:
  BridgeNode()
  : Node("vrpn_px4_bridge")
  {
    const std::string tracker = declare_parameter<std::string>("tracker", "drone1");
    const std::string in_topic =
      declare_parameter<std::string>("input_topic", "/vrpn_mocap/" + tracker + "/pose");
    const std::string out_topic =
      declare_parameter<std::string>("output_topic", "/fmu/in/vehicle_visual_odometry");

    // PX4 uXRCE-DDS topics use best-effort, keep-last QoS.
    rclcpp::QoS px4_qos(rclcpp::KeepLast(10));
    px4_qos.best_effort();

    pub_ = create_publisher<px4_msgs::msg::VehicleOdometry>(out_topic, px4_qos);
    sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      in_topic, rclcpp::SensorDataQoS(),
      std::bind(&BridgeNode::on_pose, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Relaying %s -> %s (ENU->NED)",
      in_topic.c_str(), out_topic.c_str());
  }

private:
  void on_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    const auto & p = msg->pose.position;
    const auto & q = msg->pose.orientation;

    px4_msgs::msg::VehicleOdometry odom;
    const uint64_t t_us = get_clock()->now().nanoseconds() / 1000;
    odom.timestamp = t_us;
    odom.timestamp_sample = t_us;
    odom.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;

    odom.position = enu_to_ned_pos(p.x, p.y, p.z);
    odom.q = enu_to_ned_quat(q.w, q.x, q.y, q.z);

    // We only carry pose; tell EKF2 to ignore velocity by marking it invalid.
    const float nan = std::numeric_limits<float>::quiet_NaN();
    odom.velocity = {nan, nan, nan};
    odom.angular_velocity = {nan, nan, nan};
    odom.velocity_frame = px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_UNKNOWN;

    pub_->publish(odom);
  }

  rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_;
};

}  // namespace vrpn_px4_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<vrpn_px4_bridge::BridgeNode>());
  rclcpp::shutdown();
  return 0;
}
