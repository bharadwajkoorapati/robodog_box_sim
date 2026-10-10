#include <cmath>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

class TargetCubeListener : public rclcpp::Node
{
public:
  TargetCubeListener() : Node("target_cube_listener")
  {
    // Subscribe to the target cube pose topic bridged from Gazebo
    subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/target_cube/pose",
      10,
      std::bind(&TargetCubeListener::pose_callback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(
      this->get_logger(),
      "Target Cube Listener Node started. Listening to /target_cube/pose..."
    );
  }

private:
  void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    const auto & pos = msg->pose.position;
    const auto & ori = msg->pose.orientation;

    // Convert quaternion to yaw angle (rotation about Z)
    double siny_cosp = 2.0 * (ori.w * ori.z + ori.x * ori.y);
    double cosy_cosp = 1.0 - 2.0 * (ori.y * ori.y + ori.z * ori.z);
    double yaw_rad = std::atan2(siny_cosp, cosy_cosp);
    double yaw_deg = yaw_rad * (180.0 / M_PI);

    // Throttle printing to once per second to keep terminal readable
    RCLCPP_INFO_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      1000,  // Milliseconds
      "[Target Cube Pose] Position -> (X: %.3f, Y: %.3f, Z: %.3f) | Yaw: %.2f° (qx: %.3f, qy: %.3f, qz: %.3f, qw: %.3f)",
      pos.x, pos.y, pos.z, yaw_deg, ori.x, ori.y, ori.z, ori.w
    );
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TargetCubeListener>());
  rclcpp::shutdown();
  return 0;
}
