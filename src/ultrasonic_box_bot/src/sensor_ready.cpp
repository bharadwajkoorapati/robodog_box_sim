#include <chrono>
#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/time.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

// Exits 0 once /scan carries a usable measurement, /odom has arrived and the
// odom -> base_footprint transform exists.  Exits 1 on timeout, so the launch
// file can proceed either way instead of depending on a fixed sleep.
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("sensor_ready");

  const double timeout_sec = node->declare_parameter<double>("timeout_sec", 90.0);
  const std::string scan_topic = node->declare_parameter<std::string>("scan_topic", "/scan");
  const std::string odom_topic = node->declare_parameter<std::string>("odom_topic", "/odom");
  const std::string odom_frame = node->declare_parameter<std::string>("odom_frame", "odom");
  const std::string base_frame = node->declare_parameter<std::string>("base_frame", "base_footprint");

  bool got_scan = false;
  bool got_odom = false;

  auto scan_sub = node->create_subscription<sensor_msgs::msg::LaserScan>(
    scan_topic, rclcpp::SensorDataQoS(),
    [&got_scan](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
      if (msg->ranges.empty()) {
        return;
      }
      for (float r : msg->ranges) {
        if (std::isfinite(r) && r >= msg->range_min && r <= msg->range_max) {
          got_scan = true;
          return;
        }
      }
    });

  auto odom_sub = node->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic, rclcpp::QoS(10),
    [&got_odom](nav_msgs::msg::Odometry::ConstSharedPtr) {
      got_odom = true;
    });

  tf2_ros::Buffer buffer(node->get_clock());
  tf2_ros::TransformListener listener(buffer, node, false);

  const auto start = std::chrono::steady_clock::now();
  rclcpp::WallRate rate(20.0);
  int result = 1;

  while (rclcpp::ok()) {
    rclcpp::spin_some(node);

    // Only query the buffer once both frames have been seen; canTransform() on
    // unknown frames logs a noisy "frame does not exist" warning every cycle.
    const std::string frames = buffer.allFramesAsString();
    const bool frames_known =
      frames.find(odom_frame) != std::string::npos &&
      frames.find(base_frame) != std::string::npos;
    const bool got_tf = frames_known &&
      buffer.canTransform(odom_frame, base_frame, tf2::TimePointZero);
    if (got_scan && got_odom && got_tf) {
      RCLCPP_INFO(
        node->get_logger(),
        "Sensors ready: %s data, %s data and %s -> %s transform are available",
        scan_topic.c_str(), odom_topic.c_str(), odom_frame.c_str(), base_frame.c_str());
      result = 0;
      break;
    }

    const double elapsed =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    if (elapsed > timeout_sec) {
      RCLCPP_ERROR(
        node->get_logger(),
        "Timed out after %.1fs waiting for sensors (scan=%d odom=%d tf=%d)",
        timeout_sec, got_scan, got_odom, got_tf);
      break;
    }
    rate.sleep();
  }

  rclcpp::shutdown();
  return result;
}
