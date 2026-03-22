#include "tsp_planner_node.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

TspPlannerNode::TspPlannerNode() : Node("tsp_planner_node")
{
  centroids_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "centroids", 10,
    [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
      centroids_callback(msg);
    });

  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("collection_path", 10);
}

void TspPlannerNode::centroids_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*msg, *cloud);

  std::vector<geometry_msgs::msg::Point> points;
  for (const auto& point : cloud->points) {
    geometry_msgs::msg::Point p;
    p.x = point.x;
    p.y = point.y;
    p.z = point.z;
    points.push_back(p);
  }

  auto path = solve_tsp(points);
  publish_path(path);
}

std::vector<geometry_msgs::msg::Point> TspPlannerNode::solve_tsp(const std::vector<geometry_msgs::msg::Point>& points)
{
  if (points.empty()) return {};

  // greedy TSP: start from first point, always go to nearest unvisited
  std::vector<geometry_msgs::msg::Point> path = {points[0]};
  std::vector<bool> visited(points.size(), false);
  visited[0] = true;

  while (path.size() < points.size()) {
    size_t last_idx = path.size() - 1;
    double min_dist = std::numeric_limits<double>::max();
    size_t next_idx = 0;

    for (size_t i = 0; i < points.size(); ++i) {
      if (!visited[i]) {
        double dist = std::hypot(points[last_idx].x - points[i].x, points[last_idx].y - points[i].y);
        if (dist < min_dist) {
          min_dist = dist;
          next_idx = i;
        }
      }
    }

    path.push_back(points[next_idx]);
    visited[next_idx] = true;
  }

  return path;
}

void TspPlannerNode::publish_path(const std::vector<geometry_msgs::msg::Point>& path)
{
  nav_msgs::msg::Path path_msg;
  path_msg.header.frame_id = "map";
  path_msg.header.stamp = this->now();

  for (const auto& point : path) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path_msg.header;
    pose.pose.position = point;
    pose.pose.orientation.w = 1.0;
    path_msg.poses.push_back(pose);
  }

  path_pub_->publish(path_msg);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TspPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
