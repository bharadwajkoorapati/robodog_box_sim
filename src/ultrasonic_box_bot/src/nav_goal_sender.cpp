// nav_goal_sender
// ---------------
// One-shot command-line replacement for
//
//   ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose
//     "{pose: {header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.0, z: 0.0},
//       orientation: {z: 0.0, w: 1.0}}}}"
// (on one line; the shell code above is only split here for readability)
//
// It takes the goal as plain numbers, either positionally
//
//   ros2 run ultrasonic_box_bot nav_goal_sender 4.0 4.0
//   ros2 run ultrasonic_box_bot nav_goal_sender 4.0 4.0 1.57     # x y yaw
//
// or as parameters
//
//   ros2 run ultrasonic_box_bot nav_goal_sender --ros-args -p x:=4.0 -p y:=4.0
//
// and drives the robot there "blind": the planner is configured with
// allow_unknown, so a goal in a part of the arena Nav2 has not seen yet is
// accepted and replanned as the map fills in.  The process waits for the
// NavigateToPose result and exits 0 on success, 1 on abort/cancel/timeout, so
// it can be used from scripts.

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

namespace
{
geometry_msgs::msg::Quaternion yaw_to_quaternion(double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.z = std::sin(yaw / 2.0);
  q.w = std::cos(yaw / 2.0);
  return q;
}

bool parse_double(const std::string & text, double & value)
{
  try {
    size_t consumed = 0;
    value = std::stod(text, &consumed);
    return consumed == text.size();
  } catch (const std::exception &) {
    return false;
  }
}
}  // namespace

int main(int argc, char * argv[])
{
  // init_and_remove_ros_arguments gives back only the non-ROS arguments, so
  // `--ros-args -p ...` never ends up being parsed as a coordinate.
  const auto non_ros_args = rclcpp::init_and_remove_ros_arguments(argc, argv);

  // Pure numbers are the positional goal: `nav_goal_sender 4.0 4.0 [yaw]`.
  std::vector<double> positional;
  for (const auto & arg : non_ros_args) {
    double value = 0.0;
    if (parse_double(arg, value)) {
      positional.push_back(value);
    }
  }

  auto node = std::make_shared<rclcpp::Node>("nav_goal_sender");
  const double nan = std::numeric_limits<double>::quiet_NaN();
  double x = node->declare_parameter<double>("x", nan);
  double y = node->declare_parameter<double>("y", nan);
  double yaw = node->declare_parameter<double>("yaw", 0.0);
  const std::string frame_id = node->declare_parameter<std::string>("frame_id", "map");
  const double server_timeout = node->declare_parameter<double>("server_timeout_sec", 30.0);
  const double result_timeout = node->declare_parameter<double>("result_timeout_sec", 600.0);

  if (positional.size() >= 2) {
    // Positional input wins over the parameter defaults.
    x = positional[0];
    y = positional[1];
    if (positional.size() >= 3) {
      yaw = positional[2];
    }
  }

  if (!std::isfinite(x) || !std::isfinite(y)) {
    RCLCPP_ERROR(
      node->get_logger(),
      "No goal given. Usage: nav_goal_sender <x> <y> [yaw]  or  "
      "--ros-args -p x:=4.0 -p y:=4.0 [-p yaw:=0.0]");
    rclcpp::shutdown();
    return 2;
  }

  auto client = rclcpp_action::create_client<NavigateToPose>(node, "navigate_to_pose");
  if (!client->wait_for_action_server(
      std::chrono::duration<double>(server_timeout > 0.0 ? server_timeout : 30.0)))
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "NavigateToPose action server did not appear within %.1f s. Is Nav2 "
      "running and past the costmap start-up?", server_timeout);
    rclcpp::shutdown();
    return 1;
  }

  NavigateToPose::Goal goal;
  goal.pose.header.frame_id = frame_id;
  goal.pose.header.stamp = node->now();
  goal.pose.pose.position.x = x;
  goal.pose.pose.position.y = y;
  goal.pose.pose.position.z = 0.0;
  goal.pose.pose.orientation = yaw_to_quaternion(yaw);

  rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;
  options.goal_response_callback = [node](const GoalHandle::SharedPtr & handle) {
      if (!handle) {
        RCLCPP_ERROR(node->get_logger(), "Goal rejected by the NavigateToPose server");
        return;
      }
      RCLCPP_INFO(node->get_logger(), "Goal accepted");
    };
  options.feedback_callback = [node](GoalHandle::SharedPtr, const auto feedback) {
      RCLCPP_INFO_THROTTLE(
        node->get_logger(), *node->get_clock(), 2000,
        "Navigating: %.2f m remaining", feedback->distance_remaining);
    };

  RCLCPP_INFO(
    node->get_logger(), "Sending goal (%.2f, %.2f, yaw %.2f rad) in '%s'",
    x, y, yaw, frame_id.c_str());

  auto goal_future = client->async_send_goal(goal, options);
  if (rclcpp::spin_until_future_complete(node, goal_future) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(), "Sending the goal failed");
    rclcpp::shutdown();
    return 1;
  }

  auto handle = goal_future.get();
  if (!handle) {
    rclcpp::shutdown();
    return 1;
  }

  auto result_future = client->async_get_result(handle);
  const auto waited = rclcpp::spin_until_future_complete(
    node, result_future, std::chrono::duration<double>(result_timeout));
  if (waited != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(
      node->get_logger(), "No result within %.1f s: cancelling the goal", result_timeout);
    client->async_cancel_goal(handle);
    rclcpp::shutdown();
    return 1;
  }

  const auto result = result_future.get();
  int exit_code = 1;
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(node->get_logger(), "SUCCEEDED: reached (%.2f, %.2f)", x, y);
      exit_code = 0;
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(
        node->get_logger(),
        "ABORTED: the planner or controller gave up. Check the costmap bounds "
        "and whether the goal is reachable in RViz.");
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_WARN(node->get_logger(), "CANCELED");
      break;
    default:
      RCLCPP_ERROR(node->get_logger(), "Unknown result code");
      break;
  }

  rclcpp::shutdown();
  return exit_code;
}
