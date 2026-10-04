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
        // Subscribe to the /scan topic
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 
            10, 
            std::bind(&LidarDetectorNode::scan_callback, this, std::placeholders::_1)
        );
        
        RCLCPP_INFO(this->get_logger(), "Lidar Detector Node (C++) has started and is listening to /scan...");
    }

private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) const
    {
        float angle_min = msg->angle_min;
        float angle_increment = msg->angle_increment;

        // Loop through the list of measured ranges using a basic index counter
        for (size_t i = 0; i < msg->ranges.size(); ++i)
        {
            float distance = msg->ranges[i];

            // 1. Skip invalid readings (NaN, Inf, or out-of-bound errors)
            if (std::isnan(distance) || std::isinf(distance)) {
                continue;
            }
            if (distance < msg->range_min || distance > msg->range_max) {
                continue;
            }

            // 2. Check if an obstacle is closer than the 0.5-meter threshold
            if (distance < 0.5f) {
                // 3. Associate the array index with its exact geometric angle
                float angle_rad = angle_min + (static_index_to_angle(i, angle_increment));
                float angle_deg = angle_rad * (180.0f / M_PI);

                // 4. Print the detection event
                RCLCPP_WARN(
                    this->get_logger(),
                    "Obstacle Detected! Distance: %.3fm | Angle: %.3f rad (%.1f°)",
                    distance, angle_rad, angle_deg
                );
            }
        }
    }

    // Helper method to keep line math clean
    float static_index_to_angle(size_t index, float increment) const {
        return static_cast<float>(index) * increment;
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
