#include <chrono>
#include <cmath>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class LidarDetectorNode : public rclcpp::Node
{
public:
  LidarDetectorNode() : Node("lidar_detector_node")
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 
      rclcpp::SensorDataQoS(), 
      std::bind(&LidarDetectorNode::scan_callback, this, std::placeholders::_1)
    );
    
    RCLCPP_INFO(this->get_logger(), "Lidar Detector Node listening to /scan");
  }

private:
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) const
  {
    const float angle_min = msg->angle_min;
    const float angle_increment = msg->angle_increment;

    for (size_t i = 0; i < msg->ranges.size(); ++i)
    {
      const float distance = msg->ranges[i];

      if (!std::isfinite(distance) || distance < msg->range_min || distance > msg->range_max) {
        continue;
      }

      if (distance < 0.5f) {
        const float angle_rad = angle_min + static_cast<float>(i) * angle_increment;
        const float angle_deg = angle_rad * (180.0f / static_cast<float>(M_PI));

        RCLCPP_WARN(
          this->get_logger(),
          "Obstacle detected: %.3fm at %.1f deg",
          distance, angle_deg
        );
      }
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarDetectorNode>());
  rclcpp::shutdown();
  return 0;
}
