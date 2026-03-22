#include "path_planner_node.hpp"

PathPlannerNode::PathPlannerNode() : Node("path_planner_node")
{
  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    "collection_path", 10,
    [this](const nav_msgs::msg::Path::SharedPtr msg) {
      path_callback(msg);
    });

  nav_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
    this, "navigate_to_pose");

  // Wait for the action server to be available
  if (!nav_client_->wait_for_action_server(std::chrono::seconds(10))) {
    RCLCPP_WARN(this->get_logger(), "NavigateToPose action server not available. Will retry on goal.");
  }
}

void PathPlannerNode::path_callback(const nav_msgs::msg::Path::SharedPtr msg)
{
  if (msg->poses.empty()) {
    RCLCPP_WARN(this->get_logger(), "Received empty path");
    return;
  }

  // Send goal to first pose in the path
  send_goal(msg->poses[0]);
}

void PathPlannerNode::send_goal(const geometry_msgs::msg::PoseStamped& goal_pose)
{
  if (!nav_client_->wait_for_action_server(std::chrono::seconds(5))) {
    RCLCPP_ERROR(this->get_logger(), "NavigateToPose action server not available after retry");
    return;
  }

  auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
  goal_msg.pose = goal_pose;

  auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
  send_goal_options.result_callback =
    [](const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult & result) {
      switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
          RCLCPP_INFO(rclcpp::get_logger("path_planner"), "Goal reached!");
          break;
        case rclcpp_action::ResultCode::ABORTED:
          RCLCPP_ERROR(rclcpp::get_logger("path_planner"), "Goal aborted");
          break;
        case rclcpp_action::ResultCode::CANCELED:
          RCLCPP_WARN(rclcpp::get_logger("path_planner"), "Goal canceled");
          break;
        default:
          RCLCPP_ERROR(rclcpp::get_logger("path_planner"), "Unknown result code");
          break;
      }
    };

  send_goal_options.feedback_callback =
    [](const std::shared_ptr<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>> handle,
       const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback) {
      RCLCPP_DEBUG(rclcpp::get_logger("path_planner"), "Navigation feedback received");
    };

  RCLCPP_INFO(this->get_logger(), "Sending navigation goal to (%.2f, %.2f)",
              goal_pose.pose.position.x, goal_pose.pose.position.y);

  nav_client_->async_send_goal(goal_msg, send_goal_options);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
