// map_padder
// ----------
// The global costmap's StaticLayer resizes the costmap to whatever /map
// contains.  SLAM only maps what the robot has seen, so early in a run the map
// can be a few metres wide and a goal outside it is rejected with
//   "Goal Coordinates of (4.000000, 4.000000) was outside bounds"
// Worse, every time SLAM grows the map the costmap is reallocated, which shows
// up as a glitching global costmap that seems to forget its bounds.
//
// This node republishes /map inside a fixed arena window (default 12 x 12 m,
// i.e. the 10 x 10 m world plus 1 m of margin) with everything outside the
// explored map marked unknown (-1).  The StaticLayer therefore always gets the
// same size, the costmap stops resizing, and goals anywhere in the arena are in
// bounds.  Nav2's planner has allow_unknown: true, so it will plan towards an
// unknown goal and replan as the map fills in.
//
// The arena window is expressed in the map frame (the world origin by
// convention), not relative to the map extent, precisely so the grid size is
// stable across a session.  It is widened if the map ever grows past it, so
// data is never clipped.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"

class MapPadder : public rclcpp::Node
{
public:
  MapPadder()
  : Node("map_padder")
  {
    input_topic_ = this->declare_parameter<std::string>("input_topic", "map");
    output_topic_ = this->declare_parameter<std::string>("output_topic", "map_padded");
    min_x_ = this->declare_parameter<double>("min_x", -6.0);
    max_x_ = this->declare_parameter<double>("max_x", 6.0);
    min_y_ = this->declare_parameter<double>("min_y", -6.0);
    max_y_ = this->declare_parameter<double>("max_y", 6.0);

    if (min_x_ >= max_x_ || min_y_ >= max_y_) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Invalid arena window [%.2f, %.2f] x [%.2f, %.2f]: using 12 x 12 m instead",
        min_x_, max_x_, min_y_, max_y_);
      min_x_ = min_y_ = -6.0;
      max_x_ = max_y_ = 6.0;
    }

    // The same transient-local durability as /map and the StaticLayer, so a
    // consumer that starts after the last publish still receives the grid.
    const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(output_topic_, qos);
    subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      input_topic_, qos,
      std::bind(&MapPadder::map_callback, this, std::placeholders::_1));

    RCLCPP_INFO(
      this->get_logger(),
      "Map padder: %s -> %s, arena window [%.1f, %.1f] x [%.1f, %.1f] m",
      input_topic_.c_str(), output_topic_.c_str(),
      min_x_, max_x_, min_y_, max_y_);
  }

private:
  void map_callback(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg)
  {
    const double resolution = msg->info.resolution;
    const int src_width = static_cast<int>(msg->info.width);
    const int src_height = static_cast<int>(msg->info.height);
    if (resolution <= 0.0 || src_width <= 0 || src_height <= 0) {
      RCLCPP_WARN(this->get_logger(), "Ignoring empty / invalid map (resolution %.3f)", resolution);
      return;
    }
    if (msg->data.size() < static_cast<size_t>(src_width) * src_height) {
      RCLCPP_WARN(
        this->get_logger(), "Ignoring malformed map: %zu cells for %d x %d",
        msg->data.size(), src_width, src_height);
      return;
    }

    const double src_min_x = msg->info.origin.position.x;
    const double src_min_y = msg->info.origin.position.y;
    const double src_max_x = src_min_x + src_width * resolution;
    const double src_max_y = src_min_y + src_height * resolution;

    // Never clip real data, but otherwise keep the configured window.
    double min_x = std::min(min_x_, src_min_x);
    double min_y = std::min(min_y_, src_min_y);
    double max_x = std::max(max_x_, src_max_x);
    double max_y = std::max(max_y_, src_max_y);

    // Snap outwards to the map's own resolution so the grid dimensions are
    // identical for every map that fits inside the window (no resizing, no
    // half-cell origin drift).
    min_x = std::floor(min_x / resolution) * resolution;
    min_y = std::floor(min_y / resolution) * resolution;
    max_x = std::ceil(max_x / resolution) * resolution;
    max_y = std::ceil(max_y / resolution) * resolution;

    const int width = static_cast<int>(std::lround((max_x - min_x) / resolution));
    const int height = static_cast<int>(std::lround((max_y - min_y) / resolution));
    if (width <= 0 || height <= 0) {
      return;
    }

    nav_msgs::msg::OccupancyGrid padded;
    padded.header = msg->header;
    padded.info.resolution = resolution;
    padded.info.width = static_cast<uint32_t>(width);
    padded.info.height = static_cast<uint32_t>(height);
    padded.info.origin.position.x = min_x;
    padded.info.origin.position.y = min_y;
    padded.info.origin.position.z = 0.0;
    padded.info.origin.orientation.w = 1.0;
    // -1 == unknown: exactly what an unexplored cell should be for the planner.
    padded.data.assign(static_cast<size_t>(width) * height, -1);

    const int offset_x = static_cast<int>(std::lround((src_min_x - min_x) / resolution));
    const int offset_y = static_cast<int>(std::lround((src_min_y - min_y) / resolution));
    for (int y = 0; y < src_height; ++y) {
      const int dst_y = y + offset_y;
      if (dst_y < 0 || dst_y >= height) {
        continue;
      }
      for (int x = 0; x < src_width; ++x) {
        const int dst_x = x + offset_x;
        if (dst_x < 0 || dst_x >= width) {
          continue;
        }
        padded.data[static_cast<size_t>(dst_y) * width + dst_x] =
          msg->data[static_cast<size_t>(y) * src_width + x];
      }
    }

    if (width != last_width_ || height != last_height_) {
      last_width_ = width;
      last_height_ = height;
      RCLCPP_INFO(
        this->get_logger(),
        "Publishing %d x %d cells (%.2f x %.2f m, res %.3f) padded from %d x %d; "
        "map origin (%.2f, %.2f)",
        width, height, width * resolution, height * resolution, resolution,
        src_width, src_height, src_min_x, src_min_y);
    }

    publisher_->publish(padded);
  }

  std::string input_topic_;
  std::string output_topic_;
  double min_x_{-6.0};
  double max_x_{6.0};
  double min_y_{-6.0};
  double max_y_{6.0};
  int last_width_{0};
  int last_height_{0};

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subscription_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapPadder>());
  rclcpp::shutdown();
  return 0;
}
