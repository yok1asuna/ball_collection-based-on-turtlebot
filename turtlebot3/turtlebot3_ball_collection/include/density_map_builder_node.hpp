#ifndef TURTLEBOT3_BALL_COLLECTION__DENSITY_MAP_BUILDER_NODE_HPP_
#define TURTLEBOT3_BALL_COLLECTION__DENSITY_MAP_BUILDER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class DensityMapBuilderNode : public rclcpp::Node
{
public:
  DensityMapBuilderNode();

private:
  void target_poses_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg);
  void build_density_map(const std::vector<geometry_msgs::msg::Point>& points);
  void publish_density_map();
  void send_peak_navigation_goal();

  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr target_poses_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr density_map_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_client_;

  nav_msgs::msg::OccupancyGrid density_grid_;
  double resolution_;
  int width_, height_;
  double origin_x_, origin_y_;
  int density_increment_;
  int max_density_;
  double gaussian_sigma_;
  double time_decay_factor_;
  bool navigate_on_peak_;
  double goal_republish_distance_;
  geometry_msgs::msg::Point last_goal_point_;
  bool has_last_goal_{false};
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

#endif