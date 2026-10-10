#include <algorithm>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

class KeyboardJointController : public rclcpp::Node
{
public:
  KeyboardJointController()
  : Node("keyboard_joint_controller"),
    target_position_(0.0)
  {
    key_subscriber_ =
      create_subscription<std_msgs::msg::Int32>(
      "/keyboard/keypress",
      10,
      std::bind(
        &KeyboardJointController::key_callback,
        this,
        std::placeholders::_1));

    trajectory_publisher_ =
      create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "/arm_controller/joint_trajectory",
      10);

    RCLCPP_INFO(
      get_logger(),
      "Listening to /keyboard/keypress. E increases revolute_1; Q decreases it.");
  }

private:
  void key_callback(const std_msgs::msg::Int32::SharedPtr message)
  {
    constexpr double step = 0.017;  // Step size for each key press.

    // Gazebo Key Publisher may report uppercase or lowercase ASCII.
    if (message->data == 'e' || message->data == 'E') {
      target_position_ += step;
    } else if (message->data == 'q' || message->data == 'Q') {
      target_position_ -= step;
    } else {
      return;
    }

    // Limits from the provided URDF for revolute_1.
    target_position_ = std::clamp(target_position_, -1.5714, 1.5714);

    trajectory_msgs::msg::JointTrajectory trajectory;
    trajectory.header.stamp = now();
    trajectory.joint_names = {"revolute_1"};

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = {target_position_};
    point.time_from_start.sec = 1;

    trajectory.points.push_back(point);
    trajectory_publisher_->publish(trajectory);

    RCLCPP_INFO(
      get_logger(),
      "Published revolute_1 target: %.2f rad",
      target_position_);
  }

  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr key_subscriber_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr
    trajectory_publisher_;

  double target_position_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KeyboardJointController>());
  rclcpp::shutdown();
  return 0;
}