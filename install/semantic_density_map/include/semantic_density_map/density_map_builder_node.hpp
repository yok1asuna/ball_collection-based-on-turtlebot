#ifndef SEMANTIC_DENSITY_MAP__DENSITY_MAP_BUILDER_NODE_HPP_
#define SEMANTIC_DENSITY_MAP__DENSITY_MAP_BUILDER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

class DensityMapBuilderNode : public rclcpp::Node
{
public:
  DensityMapBuilderNode();

private:
  void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void build_density_map(const std::vector<geometry_msgs::msg::Point>& points);
  void publish_density_map();

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr density_map_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  nav_msgs::msg::OccupancyGrid density_grid_;
  double resolution_;
  int width_, height_;
  double origin_x_, origin_y_;
};

#endif  // SEMANTIC_DENSITY_MAP__DENSITY_MAP_BUILDER_NODE_HPP_