#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "tf2/time.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("navigation_ready");
  const double timeout = node->declare_parameter<double>("timeout_sec", 120.0);
  tf2_ros::Buffer buffer(node->get_clock());
  tf2_ros::TransformListener listener(buffer, node, false);
  bool map_received = false;
  auto sub = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", rclcpp::QoS(1).reliable().transient_local(),
    [&map_received](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {
      map_received = !msg->data.empty() && msg->info.width > 0 && msg->info.height > 0;
    });
  const auto start = std::chrono::steady_clock::now();
  rclcpp::WallRate rate(10.0);
  int result = 1;
  while (rclcpp::ok()) {
    rclcpp::spin_some(node);
    if (map_received && buffer.canTransform("map", "base_footprint", tf2::TimePointZero)) {
      RCLCPP_INFO(node->get_logger(), "Map and map -> base_footprint TF are available");
      result = 0;
      break;
    }
    if (std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count() > timeout) {
      RCLCPP_ERROR(node->get_logger(), "Timed out waiting for /map and map -> base_footprint. For localization, set initial pose in RViz.");
      break;
    }
    rate.sleep();
  }
  rclcpp::shutdown();
  return result;
}
