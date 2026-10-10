#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using namespace std::chrono_literals;

class TargetCubePublisher : public rclcpp::Node
{
public:
  TargetCubePublisher() : Node("target_cube_publisher")
  {
    // Declare parameters with default coordinates matching empty_us_world.sdf
    this->declare_parameter<double>("target_x", 4.0);
    this->declare_parameter<double>("target_y", -4.0);
    this->declare_parameter<double>("target_z", 0.125);
    this->declare_parameter<std::string>("frame_id", "odom");
    this->declare_parameter<double>("publish_rate_hz", 10.0);

    publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/target_cube/pose", 10);

    double rate = this->get_parameter("publish_rate_hz").as_double();
    auto timer_period = std::chrono::duration<double>(1.0 / (rate > 0.0 ? rate : 10.0));

    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(timer_period),
      std::bind(&TargetCubePublisher::publish_pose, this)
    );

    RCLCPP_INFO(
      this->get_logger(),
      "Target Cube Publisher active on /target_cube/pose (X: %.3f, Y: %.3f, Z: %.3f)",
      this->get_parameter("target_x").as_double(),
      this->get_parameter("target_y").as_double(),
      this->get_parameter("target_z").as_double()
    );
  }

private:
  void publish_pose()
  {
    geometry_msgs::msg::PoseStamped msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = this->get_parameter("frame_id").as_string();

    msg.pose.position.x = this->get_parameter("target_x").as_double();
    msg.pose.position.y = this->get_parameter("target_y").as_double();
    msg.pose.position.z = this->get_parameter("target_z").as_double();

    msg.pose.orientation.x = 0.0;
    msg.pose.orientation.y = 0.0;
    msg.pose.orientation.z = 0.0;
    msg.pose.orientation.w = 1.0;

    publisher_->publish(msg);
  }

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TargetCubePublisher>());
  rclcpp::shutdown();
  return 0;
}
