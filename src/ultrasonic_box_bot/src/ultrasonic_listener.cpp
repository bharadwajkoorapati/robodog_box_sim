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
    // Publish a stop command only when explicitly enabled (default false to prevent /cmd_vel collisions)
    enable_stop_command_ = this->declare_parameter<bool>("enable_stop_command", false);
    stop_distance_ = this->declare_parameter<double>("stop_distance", 0.5);

    // Subscribe to front ultrasonic sensor
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/us_front", 
      10, // QoS History Depth
      std::bind(&UltrasonicParser::topic_callback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "Ultrasonic Parser Node started, listening to /us_front (enable_stop_command=%s)",
                enable_stop_command_ ? "true" : "false");
    cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  }

private:
  void topic_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) const
  {
    if (msg->ranges.empty()) {
      RCLCPP_WARN(this->get_logger(), "Received empty ranges array!");
      return;
    }

    std::vector<float> valid_ranges;

    for (float r : msg->ranges) {
      if (r >= msg->range_min && r <= msg->range_max && std::isfinite(r)) {
        valid_ranges.push_back(r);
      }
    }

    if (!valid_ranges.empty()) {
      float closest_obstacle = *std::min_element(valid_ranges.begin(), valid_ranges.end());
      
      RCLCPP_INFO(this->get_logger(), "Obstacle detected ahead at: %.3f meters", closest_obstacle);
      if (closest_obstacle < stop_distance_) {
        RCLCPP_WARN(this->get_logger(), "Obstacle too close (%.3f m < %.2f m)!", closest_obstacle, stop_distance_);
        if (enable_stop_command_) {
          geometry_msgs::msg::Twist stop_message;
          cmd_vel_publisher_->publish(stop_message);
        }
      }
    } 
    else {
      RCLCPP_INFO(this->get_logger(), "Path is clear! No obstacles up to %.2f meters.", msg->range_max);
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
