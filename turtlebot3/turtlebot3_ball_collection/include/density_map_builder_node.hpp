#ifndef TURTLEBOT3_BALL_COLLECTION__DENSITY_MAP_BUILDER_NODE_HPP_
#define TURTLEBOT3_BALL_COLLECTION__DENSITY_MAP_BUILDER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class DensityMapBuilderNode : public rclcpp::Node
{
public:
  DensityMapBuilderNode();

private:
  void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void build_density_map(const std::vector<geometry_msgs::msg::Point>& points);
  void publish_density_map();

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr yolo_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr density_map_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  nav_msgs::msg::OccupancyGrid density_grid_;
  double resolution_;
  int width_, height_;
  double origin_x_, origin_y_;
  int density_increment_;
  int max_density_;
  double gaussian_sigma_;
  double time_decay_factor_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

#endif  // TURTLEBOT3_BALL_COLLECTION__DENSITY_MAP_BUILDER_NODE_HPP_