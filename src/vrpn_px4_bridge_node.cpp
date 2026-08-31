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

#include <array>
#include <limits>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "rclcpp/rclcpp.hpp"

#include "vrpn_px4_bridge/frame_transforms.hpp"

namespace vrpn_px4_bridge {

class BridgeNode : public rclcpp::Node {
public:
  BridgeNode() : Node("vrpn_px4_bridge") {
    const std::string tracker = declare_parameter<std::string>("tracker", "drone1");
    const std::string in_topic =
        declare_parameter<std::string>("input_topic", "/vrpn_mocap/" + tracker + "/pose");
    const std::string out_topic =
        declare_parameter<std::string>("output_topic", "/fmu/in/vehicle_visual_odometry");

    /*
     * Optional ground-truth velocity source, off unless a topic is given.
     *
     * Intended for simulation: gz's OdometryPublisher emits exact velocity, so
     * nothing has to be differentiated. Real mocap normally supplies pose only,
     * and the pose-only default is unchanged.
     *
     * FRAME, MEASURED NOT ASSUMED. nav_msgs/Odometry puts twist in the CHILD
     * frame, and gz obeys that: a model rolled 90 deg in free fall reports its
     * fall on twist.linear.y, not .z. So this is BODY FLU, not world ENU.
     * Rotating it with the ENU->NED world map would be silently catastrophic --
     * correct at hover, divergent under motion. Instead convert FLU to FRD
     * (y and z negated) and hand PX4 VELOCITY_FRAME_BODY_FRD, letting the
     * estimator rotate it by q itself.
     *
     * EKF2 still only fuses velocity when EKF2_EV_CTRL bit 2 is set.
     */
    const std::string vel_topic = declare_parameter<std::string>("velocity_topic", "");

    // PX4 uXRCE-DDS topics use best-effort, keep-last QoS.
    rclcpp::QoS px4_qos(rclcpp::KeepLast(10));
    px4_qos.best_effort();

    pub_ = create_publisher<px4_msgs::msg::VehicleOdometry>(out_topic, px4_qos);
    sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(in_topic,
                                                                rclcpp::SensorDataQoS(),
                                                                std::bind(&BridgeNode::on_pose,
                                                                          this,
                                                                          std::placeholders::_1));

    if (!vel_topic.empty()) {
      vel_sub_ =
          create_subscription<nav_msgs::msg::Odometry>(vel_topic,
                                                       rclcpp::SensorDataQoS(),
                                                       [this](
                                                           nav_msgs::msg::Odometry::SharedPtr m) {
                                                         const auto& v = m->twist.twist.linear;
                                                         // Body FLU -> body FRD.
                                                         vel_frd_ = {static_cast<float>(v.x),
                                                                     static_cast<float>(-v.y),
                                                                     static_cast<float>(-v.z)};
                                                         have_vel_ = true;
                                                       });
    }

    RCLCPP_INFO(get_logger(), "Relaying %s -> %s (ENU->NED)", in_topic.c_str(), out_topic.c_str());
    RCLCPP_INFO(get_logger(),
                "Velocity: %s",
                vel_topic.empty() ? "disabled (pose only, NaN)"
                                  : ("from " + vel_topic + " as BODY_FRD").c_str());
  }

private:
  void on_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    const auto& p = msg->pose.position;
    const auto& q = msg->pose.orientation;

    px4_msgs::msg::VehicleOdometry odom;
    const uint64_t t_us = get_clock()->now().nanoseconds() / 1000;
    odom.timestamp = t_us;
    odom.timestamp_sample = t_us;
    odom.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;

    odom.position = sanitize_pos(enu_to_ned_pos(p.x, p.y, p.z));
    odom.q = normalize_or_invalidate(enu_to_ned_quat(q.w, q.x, q.y, q.z));

    const float nan = std::numeric_limits<float>::quiet_NaN();

    /* Angular rates are never carried: differentiating the mocap quaternion is
     * noisier still, and EKF2 does not fuse EV angular velocity. */
    odom.angular_velocity = {nan, nan, nan};

    if (have_vel_) {
      odom.velocity = vel_frd_;
      odom.velocity_frame = px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_BODY_FRD;
    } else {
      odom.velocity = {nan, nan, nan};
      odom.velocity_frame = px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_UNKNOWN;
    }

    pub_->publish(odom);
  }

  rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr vel_sub_;
  std::array<float, 3> vel_frd_{0.0f, 0.0f, 0.0f};
  bool have_vel_{false};
};

}  // namespace vrpn_px4_bridge

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<vrpn_px4_bridge::BridgeNode>());
  rclcpp::shutdown();
  return 0;
}
