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
    // 1. Subscribe to the specific front sensor topic.
    // The topic matches the ${name} attribute given in your xacro macro.
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/us_front", 
      10, // QoS History Depth
      std::bind(&UltrasonicParser::topic_callback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "Ultrasonic Parser Node started, listening to /us_front");
    cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  }

private:
  void topic_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) const
  {
    // 2. Check if the array contains data to avoid runtime errors
    if (msg->ranges.empty()) {
      RCLCPP_WARN(this->get_logger(), "Received empty ranges array!");
      return;
    }

    std::vector<float> valid_ranges;

    // 3. Clean up inf/nan values and extract safe readings
    for (float r : msg->ranges) {
      // Filter out values below min, above max, or math oddities (inf/nan)
      if (r >= msg->range_min && r <= msg->range_max && std::isfinite(r)) {
        valid_ranges.push_back(r);
      }
    }

    // 4. Determine your final front distance
    if (!valid_ranges.empty()) {
      // Find the closest obstacle among the 5 simulation beams
      float closest_obstacle = *std::min_element(valid_ranges.begin(), valid_ranges.end());
      
      RCLCPP_INFO(this->get_logger(), "Obstacle detected ahead at: %.3f meters", closest_obstacle);
      if (closest_obstacle < 0.5) {
        RCLCPP_WARN(this->get_logger(), "Obstacle too close! Stopping robot.");
        geometry_msgs::msg::Twist stop_message;
        cmd_vel_publisher_->publish(stop_message);
      }
    } 
    else {
      // If the list is empty, it means the simulator is flooding the array with 'inf'
      // because there are no physical obstacles within range_max.
      RCLCPP_INFO(this->get_logger(), "Path is clear! No obstacles up to %.2f meters.", msg->range_max);
    }
  }

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
