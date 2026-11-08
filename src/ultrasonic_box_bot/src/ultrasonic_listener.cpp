#include <memory>
#include <cmath>
#include <algorithm>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class UltrasonicParser : public rclcpp::Node
{
public:
  UltrasonicParser() : Node("ultrasonic_parser")
  {
    enable_stop_command_ = this->declare_parameter<bool>("enable_stop_command", false);
    stop_distance_ = this->declare_parameter<double>("stop_distance", 0.5);

    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/us_front", 
      rclcpp::SensorDataQoS(),
      std::bind(&UltrasonicParser::topic_callback, this, std::placeholders::_1)
    );

    cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    RCLCPP_INFO(this->get_logger(), "Ultrasonic parser started on /us_front (stop_enabled=%s)",
                enable_stop_command_ ? "true" : "false");
  }

private:
  void topic_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) const
  {
    if (msg->ranges.empty()) {
      return;
    }

    std::vector<float> valid_ranges;
    for (float r : msg->ranges) {
      if (std::isfinite(r) && r >= msg->range_min && r <= msg->range_max) {
        valid_ranges.push_back(r);
      }
    }

    if (valid_ranges.empty()) {
      return;
    }

    float closest = *std::min_element(valid_ranges.begin(), valid_ranges.end());
    if (closest < stop_distance_) {
      RCLCPP_WARN(this->get_logger(), "Obstacle close: %.2f m", closest);
      if (enable_stop_command_) {
        geometry_msgs::msg::Twist stop_msg;
        cmd_vel_publisher_->publish(stop_msg);
      }
    }
  }

  bool enable_stop_command_{false};
  double stop_distance_{0.5};
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UltrasonicParser>());
  rclcpp::shutdown();
  return 0;
}
