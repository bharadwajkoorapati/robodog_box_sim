#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"

using namespace std::chrono_literals;

enum class NavState {
  START_FORWARD_WALL_1,
  TURN_RIGHT_WALL_1,
  FOLLOW_WALL_1,
  CLEAR_CORNER_1,
  TURN_LEFT_GAP_1,
  FORWARD_TO_WALL_2,
  TURN_LEFT_WALL_2,
  FOLLOW_WALL_2,
  CLEAR_CORNER_2,
  TURN_RIGHT_GAP_2,
  FORWARD_TO_PERIMETER,
  TURN_RIGHT_PASSAGE,
  FOLLOW_PASSAGE_TO_TARGET,
  TARGET_REACHED
};

class AutonavigatorNode : public rclcpp::Node
{
public:
  AutonavigatorNode()
  : Node("autonavigator_node"),
    current_state_(NavState::START_FORWARD_WALL_1),
    front_dist_(10.0f),
    left_dist_(10.0f),
    right_dist_(10.0f),
    robot_x_(0.0),
    robot_y_(0.0),
    robot_yaw_(0.0),
    has_odom_(false),
    has_target_(false),
    has_front_(false), 
    has_left_(false), 
    has_right_(false),
    state_start_time_(this->now())
  {
    // Publishers
    this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      this->get_parameter("cmd_vel_topic").as_string(), 10);
    // Subscribers
    us_front_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/us_front", 10,
      std::bind(&AutonavigatorNode::front_callback, this, std::placeholders::_1));

    us_left_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/us_left", 10,
      std::bind(&AutonavigatorNode::left_callback, this, std::placeholders::_1));

    us_right_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/us_right", 10,
      std::bind(&AutonavigatorNode::right_callback, this, std::placeholders::_1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      std::bind(&AutonavigatorNode::odom_callback, this, std::placeholders::_1));

    target_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/target_cube/pose", 10,
      std::bind(&AutonavigatorNode::target_callback, this, std::placeholders::_1));

    // High frequency control loop: 20Hz (50ms)
    control_timer_ = this->create_wall_timer(
      50ms, std::bind(&AutonavigatorNode::control_loop, this));

    RCLCPP_INFO(this->get_logger(), "Autonavigator Node initialized. Starting state machine...");
  }

private:
  float extract_min_range(const sensor_msgs::msg::LaserScan::ConstSharedPtr & msg)
  {
    float min_r = 10.0f;
    for (float r : msg->ranges) {
      if (std::isfinite(r) && r >= msg->range_min && r <= msg->range_max) {
        if (r < min_r) {
          min_r = r;
        }
      }
    }
    return min_r;
  }

  void front_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr msg)
  { front_dist_ = extract_min_range(msg); has_front_ = true; }
  void left_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr msg)
  { left_dist_ = extract_min_range(msg); has_left_ = true; }
  void right_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr msg)
  { right_dist_ = extract_min_range(msg); has_right_ = true; }

  void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
  {
    robot_x_ = msg->pose.pose.position.x;
    robot_y_ = msg->pose.pose.position.y;

    const auto & q = msg->pose.pose.orientation;
    double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    robot_yaw_ = std::atan2(siny_cosp, cosy_cosp);

    has_odom_ = true;
  }

  void target_callback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
  {
    target_pos_ = msg->pose.position;
    has_target_ = true;
  }

  double angle_diff(double target, double current)
  {
    double diff = target - current;
    while (diff > M_PI) diff -= 2.0 * M_PI;
    while (diff < -M_PI) diff += 2.0 * M_PI;
    return diff;
  }

  void transition_to(NavState next_state, const std::string & state_name)
  {
    current_state_ = next_state;
    state_start_time_ = this->now();
    RCLCPP_INFO(this->get_logger(), "[State Transition] -> %s (X: %.2f, Y: %.2f, Yaw: %.1f deg)",
      state_name.c_str(), robot_x_, robot_y_, robot_yaw_ * 180.0 / M_PI);
  }

  void control_loop()
  {
    if (!has_odom_ || !has_front_ || !has_left_ || !has_right_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "Waiting for odom and ultrasonic data (odom=%d F=%d L=%d R=%d)",
        has_odom_, has_front_, has_left_, has_right_);
      cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
      return;
    }

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = 0.0;
    cmd.angular.z = 0.0;

    switch (current_state_) {
      // 1. Move forward until detecting Wall 1
      case NavState::START_FORWARD_WALL_1:
      {
        if (front_dist_ < 0.60f) {
          transition_to(NavState::TURN_RIGHT_WALL_1, "TURN_RIGHT_WALL_1");
        } else {
          cmd.linear.x = 0.25;
          // Maintain 0 heading
          double yaw_err = angle_diff(0.0, robot_yaw_);
          cmd.angular.z = std::clamp(yaw_err * 0.8, -0.3, 0.3);
        }
        break;
      }

      // 2. Turn right 90 degrees (Target Yaw: -PI/2)
      case NavState::TURN_RIGHT_WALL_1:
      {
        double target_yaw = -M_PI / 2.0;
        double diff = angle_diff(target_yaw, robot_yaw_);
        if (std::abs(diff) < 0.08) {
          transition_to(NavState::FOLLOW_WALL_1, "FOLLOW_WALL_1");
        } else {
          cmd.angular.z = (diff > 0) ? 0.5 : -0.5;
        }
        break;
      }

      // 3. Follow Wall 1 on left side (heading -Y)
      case NavState::FOLLOW_WALL_1:
      {
        // Check if wall has ended (gap at Y < -3.0 or left distance opening)
        if (left_dist_ > 1.4f || robot_y_ < -3.15) {
          transition_to(NavState::CLEAR_CORNER_1, "CLEAR_CORNER_1");
          break;
        }

        cmd.linear.x = 0.25;
        // Wall follow on Left (nominal dist: ~0.50m - 0.70m)
        if (left_dist_ < 0.45f) {
          cmd.angular.z = -0.25; // Steer right (away from wall)
        } else if (left_dist_ > 0.70f) {
          cmd.angular.z = 0.25;  // Steer left (closer to wall)
        } else {
          // Align heading with -PI/2
          double yaw_err = angle_diff(-M_PI / 2.0, robot_yaw_);
          cmd.angular.z = std::clamp(yaw_err * 0.8, -0.3, 0.3);
        }
        break;
      }

      // Short advance to clear corner 1
      case NavState::CLEAR_CORNER_1:
      {
        auto elapsed = (this->now() - state_start_time_).seconds();
        if (elapsed > 1.6) {
          transition_to(NavState::TURN_LEFT_GAP_1, "TURN_LEFT_GAP_1");
        } else {
          cmd.linear.x = 0.25;
        }
        break;
      }

      // 4. Turn left 90 degrees to face next wall (+X, Target Yaw: 0.0)
      case NavState::TURN_LEFT_GAP_1:
      {
        double target_yaw = 0.0;
        double diff = angle_diff(target_yaw, robot_yaw_);
        if (std::abs(diff) < 0.08) {
          transition_to(NavState::FORWARD_TO_WALL_2, "FORWARD_TO_WALL_2");
        } else {
          cmd.angular.z = (diff > 0) ? 0.5 : -0.5;
        }
        break;
      }

      // 5. Move through gap towards Wall 2
      case NavState::FORWARD_TO_WALL_2:
      {
        if (front_dist_ < 0.60f) {
          transition_to(NavState::TURN_LEFT_WALL_2, "TURN_LEFT_WALL_2");
        } else {
          cmd.linear.x = 0.25;
          double yaw_err = angle_diff(0.0, robot_yaw_);
          cmd.angular.z = std::clamp(yaw_err * 0.8, -0.3, 0.3);
        }
        break;
      }

      // 6. Turn left 90 degrees (Target Yaw: +PI/2, facing +Y)
      case NavState::TURN_LEFT_WALL_2:
      {
        double target_yaw = M_PI / 2.0;
        double diff = angle_diff(target_yaw, robot_yaw_);
        if (std::abs(diff) < 0.08) {
          transition_to(NavState::FOLLOW_WALL_2, "FOLLOW_WALL_2");
        } else {
          cmd.angular.z = (diff > 0) ? 0.5 : -0.5;
        }
        break;
      }

      // 7. Follow Wall 2 on right side (heading +Y)
      case NavState::FOLLOW_WALL_2:
      {
        // Check if wall has ended (gap at Y > 3.0 or right distance opening)
        if (right_dist_ > 1.4f || robot_y_ > 3.15) {
          transition_to(NavState::CLEAR_CORNER_2, "CLEAR_CORNER_2");
          break;
        }

        cmd.linear.x = 0.25;
        // Wall follow on Right (nominal dist: ~0.50m - 0.70m)
        if (right_dist_ < 0.45f) {
          cmd.angular.z = 0.25;  // Steer left (away from wall)
        } else if (right_dist_ > 0.70f) {
          cmd.angular.z = -0.25; // Steer right (closer to wall)
        } else {
          // Align heading with +PI/2
          double yaw_err = angle_diff(M_PI / 2.0, robot_yaw_);
          cmd.angular.z = std::clamp(yaw_err * 0.8, -0.3, 0.3);
        }
        break;
      }

      // Short advance to clear corner 2
      case NavState::CLEAR_CORNER_2:
      {
        auto elapsed = (this->now() - state_start_time_).seconds();
        if (elapsed > 1.6) {
          transition_to(NavState::TURN_RIGHT_GAP_2, "TURN_RIGHT_GAP_2");
        } else {
          cmd.linear.x = 0.25;
        }
        break;
      }

      // 8. Turn right 90 degrees to face perimeter (+X, Target Yaw: 0.0)
      case NavState::TURN_RIGHT_GAP_2:
      {
        double target_yaw = 0.0;
        double diff = angle_diff(target_yaw, robot_yaw_);
        if (std::abs(diff) < 0.08) {
          transition_to(NavState::FORWARD_TO_PERIMETER, "FORWARD_TO_PERIMETER");
        } else {
          cmd.angular.z = (diff > 0) ? 0.5 : -0.5;
        }
        break;
      }

      // 9. Move forward to perimeter wall
      case NavState::FORWARD_TO_PERIMETER:
      {
        if (front_dist_ < 0.60f) {
          transition_to(NavState::TURN_RIGHT_PASSAGE, "TURN_RIGHT_PASSAGE");
        } else {
          cmd.linear.x = 0.25;
          double yaw_err = angle_diff(0.0, robot_yaw_);
          cmd.angular.z = std::clamp(yaw_err * 0.8, -0.3, 0.3);
        }
        break;
      }

      // 10. Turn right 90 degrees into passage (Target Yaw: -PI/2)
      case NavState::TURN_RIGHT_PASSAGE:
      {
        double target_yaw = -M_PI / 2.0;
        double diff = angle_diff(target_yaw, robot_yaw_);
        if (std::abs(diff) < 0.08) {
          transition_to(NavState::FOLLOW_PASSAGE_TO_TARGET, "FOLLOW_PASSAGE_TO_TARGET");
        } else {
          cmd.angular.z = (diff > 0) ? 0.5 : -0.5;
        }
        break;
      }

      // 11. Follow passage to target cube (at X: 4.0, Y: -4.0)
      case NavState::FOLLOW_PASSAGE_TO_TARGET:
      {
        double target_x = has_target_ ? target_pos_.x : 4.0;
        double target_y = has_target_ ? target_pos_.y : -4.0;
        double dist_to_target = std::hypot(target_x - robot_x_, target_y - robot_y_);

        if (dist_to_target < 0.50 || front_dist_ < 0.45f || robot_y_ <= -3.85) {
          transition_to(NavState::TARGET_REACHED, "TARGET_REACHED");
          break;
        }

        cmd.linear.x = 0.25;
        // Keep centered between surrounding wall (left) and maze wall 2 (right)
        if (right_dist_ < 0.40f) {
          cmd.angular.z = 0.20;  // Steer left away from right wall
        } else if (left_dist_ < 0.40f) {
          cmd.angular.z = -0.20; // Steer right away from left wall
        } else {
          double yaw_err = angle_diff(-M_PI / 2.0, robot_yaw_);
          cmd.angular.z = std::clamp(yaw_err * 0.8, -0.3, 0.3);
        }
        break;
      }

      // 12. Mission Completed
      case NavState::TARGET_REACHED:
      {
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
        RCLCPP_INFO_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          ">>> TARGET REACHED! Mission Accomplished at (X: %.2f, Y: %.2f) <<<",
          robot_x_, robot_y_);
        break;
      }
    }

    cmd_vel_pub_->publish(cmd);
  }

  // Node members
  NavState current_state_;
  float front_dist_;
  float left_dist_;
  float right_dist_;

  double robot_x_;
  double robot_y_;
  double robot_yaw_;

  bool has_odom_;
  bool has_target_;
  bool has_front_, has_left_, has_right_;
  geometry_msgs::msg::Point target_pos_;
  rclcpp::Time state_start_time_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr us_front_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr us_left_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr us_right_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_sub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AutonavigatorNode>());
  rclcpp::shutdown();
  return 0;
}
