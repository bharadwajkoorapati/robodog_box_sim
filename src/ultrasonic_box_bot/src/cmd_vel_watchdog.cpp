// cmd_vel_watchdog
// ----------------
// Gazebo's DiffDrive plugin keeps applying the last Twist forever when commands
// stop arriving, so an aborted Nav2 goal (or a crashed controller node) leaves
// the robot rolling.  This node sits between every /cmd_vel producer and the
// Gazebo bridge:
//
//   /cmd_vel  (Nav2, autonavigator, manual teleop, ultrasonic_listener)
//        |
//        v  cmd_vel_watchdog  -- forwards every command, republishes Twist(0)
//        |                       when the stream goes stale
//        v
//   /cmd_vel_safe  -->  ros_gz_bridge  -->  DiffDrive plugin
//
// Keeping the input and output topics distinct matters: if the node published
// zeros back onto its own input topic it would re-trigger its own freshness
// timer and oscillate between forwarding and filling zeros.

#include <chrono>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

class CmdVelWatchdog : public rclcpp::Node
{
public:
  CmdVelWatchdog()
  : Node("cmd_vel_watchdog")
  {
    input_topic_ = this->declare_parameter<std::string>("input_topic", "/cmd_vel");
    output_topic_ = this->declare_parameter<std::string>("output_topic", "/cmd_vel_safe");
    timeout_sec_ = this->declare_parameter<double>("timeout_sec", 0.5);
    zero_publish_rate_hz_ = this->declare_parameter<double>("zero_publish_rate_hz", 10.0);

    if (input_topic_ == output_topic_) {
      RCLCPP_ERROR(
        this->get_logger(),
        "input_topic and output_topic are both '%s': the node would consume its own "
        "zero commands. Falling back to '/cmd_vel_safe' for the output.",
        input_topic_.c_str());
      output_topic_ = "/cmd_vel_safe";
    }

    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(output_topic_, 10);

    subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
      input_topic_, rclcpp::QoS(10),
      std::bind(&CmdVelWatchdog::cmd_callback, this, std::placeholders::_1));

    const double rate = zero_publish_rate_hz_ > 0.0 ? zero_publish_rate_hz_ : 10.0;
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / rate)),
      std::bind(&CmdVelWatchdog::check_stream, this));

    last_cmd_time_ = this->now();

    RCLCPP_INFO(
      this->get_logger(),
      "Command watchdog: %s -> %s (zero Twist after %.2f s of silence)",
      input_topic_.c_str(), output_topic_.c_str(), timeout_sec_);
  }

private:
  void cmd_callback(const geometry_msgs::msg::Twist::ConstSharedPtr msg)
  {
    publisher_->publish(*msg);
    last_cmd_time_ = this->now();
    have_input_ = true;
    if (stream_was_stale_) {
      stream_was_stale_ = false;
      RCLCPP_INFO(this->get_logger(), "Command stream resumed");
    }
  }

  void check_stream()
  {
    // Never publish before the first command: at start-up nothing should be
    // forcing a zero Twist into the bridge.
    if (!have_input_) {
      return;
    }

    const rclcpp::Time now = this->now();
    if (now < last_cmd_time_) {
      // Sim clock went backwards (restarted world); treat the stream as fresh.
      last_cmd_time_ = now;
      return;
    }

    if ((now - last_cmd_time_).seconds() <= timeout_sec_) {
      return;
    }

    geometry_msgs::msg::Twist stop;
    publisher_->publish(stop);

    if (!stream_was_stale_) {
      stream_was_stale_ = true;
      RCLCPP_WARN(
        this->get_logger(),
        "No command on %s for %.2f s: publishing zero velocity to stop the robot",
        input_topic_.c_str(), timeout_sec_);
    }
  }

  std::string input_topic_;
  std::string output_topic_;
  double timeout_sec_{0.5};
  double zero_publish_rate_hz_{10.0};

  bool have_input_{false};
  bool stream_was_stale_{false};
  rclcpp::Time last_cmd_time_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdVelWatchdog>());
  rclcpp::shutdown();
  return 0;
}
