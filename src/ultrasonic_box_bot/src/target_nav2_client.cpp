// target_nav2_client
// ------------------
// Closes the gap between "the target cube pose is published" and "Nav2 drives
// to it".  The node:
//   * subscribes to the ground-truth target pose (/target_cube/pose),
//   * transforms it from its own frame (normally `odom`) into the navigation
//     global frame (`map`) using tf2,
//   * sends exactly one NavigateToPose goal per target change (a 10 Hz target
//     publisher must not turn into 10 goals per second),
//   * ignores a repeat of the goal it is already executing,
//   * logs acceptance, feedback and the terminal result so an aborted goal is
//     visible instead of silent.
//
// Because the target is published continuously, goal de-duplication happens
// here rather than relying on the publisher being latched/one-shot.

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <optional>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2/exceptions.h"
#include "tf2/time.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

namespace
{
double yaw_from_quaternion(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}
}  // namespace

class TargetNav2Client : public rclcpp::Node
{
public:
  TargetNav2Client()
  : Node("target_nav2_client")
  {
    target_topic_ = this->declare_parameter<std::string>("target_topic", "/target_cube/pose");
    action_name_ = this->declare_parameter<std::string>("action_name", "/navigate_to_pose");
    global_frame_ = this->declare_parameter<std::string>("global_frame", "map");
    min_goal_distance_ = this->declare_parameter<double>("min_goal_distance", 0.25);
    min_goal_yaw_ = this->declare_parameter<double>("min_goal_yaw", 0.25);
    tf_timeout_sec_ = this->declare_parameter<double>("tf_timeout_sec", 1.0);
    retry_period_sec_ = this->declare_parameter<double>("retry_period_sec", 1.0);
    start_delay_sec_ = this->declare_parameter<double>("start_delay_sec", 5.0);
    cancel_on_new_target_ = this->declare_parameter<bool>("cancel_on_new_target", false);
    reject_retry_sec_ = this->declare_parameter<double>("reject_retry_sec", 5.0);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    // TransformListener is neither copyable nor movable (it owns a thread and a
    // reference to the buffer), so it cannot be a plain member; hold it by
    // pointer and construct it once the buffer exists.
    //
    // spin_thread is left at its default (true) on purpose: to_global_frame()
    // calls transform() with a timeout, and tf2 refuses timed lookups on a
    // buffer that is not fed by a dedicated thread (the lookup would then
    // always time out in our single-threaded executor).
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_, this);
    action_client_ = rclcpp_action::create_client<NavigateToPose>(this, action_name_);

    target_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      target_topic_, rclcpp::QoS(10),
      std::bind(&TargetNav2Client::target_callback, this, std::placeholders::_1));

    const double period = retry_period_sec_ > 0.0 ? retry_period_sec_ : 1.0;
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(period)),
      std::bind(&TargetNav2Client::tick, this));

    start_time_ = this->now();

    RCLCPP_INFO(
      this->get_logger(),
      "Target -> Nav2 client ready: %s -> action %s (global frame '%s')",
      target_topic_.c_str(), action_name_.c_str(), global_frame_.c_str());
  }

private:
  // ---------------------------------------------------------------- target
  void target_callback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
  {
    latest_target_ = *msg;

    // A target repeat that is (almost) the pose we last commanded is not a new
    // goal.  Without this check a 10 Hz publisher would flood the action
    // server with identical goals.
    //
    // The comparison is done in the target's own frame on purpose: the same
    // ground-truth target maps to slowly changing `map` coordinates while SLAM
    // refines map -> odom, and comparing in `map` would resend goals forever.
    if (has_last_goal_ && !target_differs_from_last(*msg)) {
      return;
    }

    if (!pending_target_) {
      pending_target_ = true;
      RCLCPP_INFO(
        this->get_logger(),
        "New target received: (%.3f, %.3f, yaw %.1f deg) in frame '%s'",
        msg->pose.position.x, msg->pose.position.y,
        yaw_from_quaternion(msg->pose.orientation) * 180.0 / M_PI,
        msg->header.frame_id.c_str());
    }

    // Optionally preempt an in-flight goal when the target moves.
    if (cancel_on_new_target_ && goal_active_ && !cancel_requested_) {
      RCLCPP_INFO(this->get_logger(), "Target changed -> cancelling the active goal");
      action_client_->async_cancel_all_goals();
      cancel_requested_ = true;
    }
  }

  bool target_differs_from_last(const geometry_msgs::msg::PoseStamped & msg) const
  {
    if (msg.header.frame_id != last_target_pose_.header.frame_id) {
      return true;
    }
    const double dx = msg.pose.position.x - last_target_pose_.pose.position.x;
    const double dy = msg.pose.position.y - last_target_pose_.pose.position.y;
    if (std::hypot(dx, dy) > min_goal_distance_) {
      return true;
    }
    const double dyaw = std::remainder(
      yaw_from_quaternion(msg.pose.orientation) -
      yaw_from_quaternion(last_target_pose_.pose.orientation), 2.0 * M_PI);
    return std::abs(dyaw) > min_goal_yaw_;
  }

  // ------------------------------------------------------------------ tick
  void tick()
  {
    if (!pending_target_ || goal_active_ || !latest_target_.has_value()) {
      return;
    }
    if ((this->now() - start_time_).seconds() < start_delay_sec_) {
      return;
    }
    if (this->now() < retry_after_) {
      return;
    }
    if (!action_client_->action_server_is_ready()) {
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "Waiting for the %s action server (Nav2 not active yet)", action_name_.c_str());
      return;
    }

    geometry_msgs::msg::PoseStamped goal_pose;
    if (!to_global_frame(*latest_target_, goal_pose)) {
      return;
    }

    NavigateToPose::Goal goal;
    goal.pose = goal_pose;

    rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;
    options.goal_response_callback =
      std::bind(&TargetNav2Client::goal_response_callback, this, std::placeholders::_1);
    options.feedback_callback =
      std::bind(
      &TargetNav2Client::feedback_callback, this,
      std::placeholders::_1, std::placeholders::_2);
    options.result_callback =
      std::bind(&TargetNav2Client::result_callback, this, std::placeholders::_1);

    RCLCPP_INFO(
      this->get_logger(),
      "Sending NavigateToPose goal: (%.2f, %.2f) in '%s'",
      goal_pose.pose.position.x, goal_pose.pose.position.y,
      goal_pose.header.frame_id.c_str());

    action_client_->async_send_goal(goal, options);

    pending_target_ = false;
    goal_active_ = true;
    cancel_requested_ = false;
    // Remember the target as published (source frame), not the transformed
    // goal, so map drift does not look like a new target.
    last_target_pose_ = *latest_target_;
    has_last_goal_ = true;
  }

  bool to_global_frame(
    const geometry_msgs::msg::PoseStamped & in,
    geometry_msgs::msg::PoseStamped & out)
  {
    if (in.header.frame_id.empty() || in.header.frame_id == global_frame_) {
      out = in;
      out.header.frame_id = global_frame_;
      out.header.stamp = this->now();
      return true;
    }

    try {
      out = tf_buffer_->transform(
        in, global_frame_, tf2::durationFromSec(tf_timeout_sec_));
    } catch (const tf2::TransformException & ex) {
      // Expected while the map frame is being created (SLAM/AMCL start-up).
      // Retried on the next tick; the target keeps arriving so no data is lost.
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 3000,
        "Cannot transform target from '%s' to '%s' yet: %s",
        in.header.frame_id.c_str(), global_frame_.c_str(), ex.what());
      return false;
    }

    out.header.stamp = this->now();
    return true;
  }

  // -------------------------------------------------------------- callbacks
  void goal_response_callback(const GoalHandleNavigateToPose::SharedPtr & goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Goal was rejected by %s (planner not ready / goal outside the costmap?); "
        "retrying in %.0f s",
        action_name_.c_str(), reject_retry_sec_);
      goal_active_ = false;
      pending_target_ = true;
      // Back off so a permanently unreachable target does not log once a second.
      retry_after_ = this->now() + rclcpp::Duration::from_seconds(reject_retry_sec_);
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Goal accepted by NavigateToPose");
  }

  void feedback_callback(
    GoalHandleNavigateToPose::SharedPtr,
    const std::shared_ptr<const NavigateToPose::Feedback> feedback)
  {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "Navigating: %.2f m remaining", feedback->distance_remaining);
  }

  void result_callback(const GoalHandleNavigateToPose::WrappedResult & result)
  {
    goal_active_ = false;
    cancel_requested_ = false;

    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), "NavigateToPose SUCCEEDED: target reached");
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(
          this->get_logger(),
          "NavigateToPose ABORTED: goal unreachable or behaviour failed; not retrying the same target");
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_WARN(this->get_logger(), "NavigateToPose CANCELED");
        break;
      default:
        RCLCPP_WARN(this->get_logger(), "NavigateToPose finished with unknown result code");
        break;
    }

    // If a new target arrived while the goal was running, tick() picks it up.
    if (latest_target_.has_value() && !target_differs_from_last(*latest_target_)) {
      pending_target_ = false;
    }
  }

  std::string target_topic_;
  std::string action_name_;
  std::string global_frame_;
  double min_goal_distance_{0.25};
  double min_goal_yaw_{0.25};
  double tf_timeout_sec_{1.0};
  double retry_period_sec_{1.0};
  double start_delay_sec_{5.0};
  bool cancel_on_new_target_{false};
  double reject_retry_sec_{5.0};

  bool pending_target_{false};
  bool goal_active_{false};
  bool cancel_requested_{false};
  bool has_last_goal_{false};

  rclcpp::Time start_time_;
  rclcpp::Time retry_after_{0, 0, RCL_ROS_TIME};
  geometry_msgs::msg::PoseStamped last_target_pose_;
  std::optional<geometry_msgs::msg::PoseStamped> latest_target_;

  tf2_ros::Buffer::SharedPtr tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TargetNav2Client>());
  rclcpp::shutdown();
  return 0;
}
