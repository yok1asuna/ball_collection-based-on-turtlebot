#ifndef SEMANTIC_DENSITY_MAP__SEMANTIC_PATH_PLANNER_NODE_HPP_
#define SEMANTIC_DENSITY_MAP__SEMANTIC_PATH_PLANNER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

class SemanticPathPlannerNode : public rclcpp::Node
{
public:
  SemanticPathPlannerNode();

private:
  void density_map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void plan_path();
  std::vector<geometry_msgs::msg::Point> kmeans_clustering(const std::vector<geometry_msgs::msg::Point>& points, int k);
  double potential_field_cost(int x, int y, const nav_msgs::msg::OccupancyGrid& grid);
  nav_msgs::msg::Path astar_path_planning(const geometry_msgs::msg::Point& start, const geometry_msgs::msg::Point& goal, const nav_msgs::msg::OccupancyGrid& grid);
  void send_navigation_goal(const geometry_msgs::msg::PoseStamped& goal);

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr density_map_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_client_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  nav_msgs::msg::OccupancyGrid current_density_map_;
  bool map_received_;
};

#endif  // SEMANTIC_DENSITY_MAP__SEMANTIC_PATH_PLANNER_NODE_HPP_