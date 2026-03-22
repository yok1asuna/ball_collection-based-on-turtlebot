#ifndef TURTLEBOT3_BALL_COLLECTION__PATH_PLANNER_NODE_HPP_
#define TURTLEBOT3_BALL_COLLECTION__PATH_PLANNER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

class PathPlannerNode : public rclcpp::Node
{
public:
  PathPlannerNode();

private:
  void path_callback(const nav_msgs::msg::Path::SharedPtr msg);
  void send_goal(const geometry_msgs::msg::PoseStamped& goal_pose);

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_client_;
};

#endif