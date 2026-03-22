#include "semantic_path_planner_node.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <queue>
#include <vector>
#include <cmath>
#include <limits>
#include <algorithm>

SemanticPathPlannerNode::SemanticPathPlannerNode() : Node("semantic_path_planner_node")
{
  density_map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "density_map", 10,
    std::bind(&SemanticPathPlannerNode::density_map_callback, this, std::placeholders::_1));

  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("semantic_path", 10);
  centroids_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("centroids", 10);

  nav_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
    this, "navigate_to_pose");

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  map_received_ = false;
}

void SemanticPathPlannerNode::density_map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  current_density_map_ = *msg;
  map_received_ = true;
  plan_path();
}

void SemanticPathPlannerNode::plan_path()
{
  if (!map_received_) return;

  // Extract high-density points
  std::vector<geometry_msgs::msg::Point> high_density_points;
  for (int i = 0; i < current_density_map_.info.width; ++i) {
    for (int j = 0; j < current_density_map_.info.height; ++j) {
      int index = j * current_density_map_.info.width + i;
      if (current_density_map_.data[index] > 50) {  // Threshold for high density
        geometry_msgs::msg::Point p;
        p.x = current_density_map_.info.origin.position.x + i * current_density_map_.info.resolution;
        p.y = current_density_map_.info.origin.position.y + j * current_density_map_.info.resolution;
        p.z = 0.0;
        high_density_points.push_back(p);
      }
    }
  }

  if (high_density_points.empty()) return;

  // K-Means clustering to find centroids
  auto centroids = kmeans_clustering(high_density_points, 3);  // Assume 3 clusters

  // Publish centroids as PointCloud2
  if (!centroids.empty()) {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    for (const auto& centroid : centroids) {
      pcl::PointXYZ point(centroid.x, centroid.y, centroid.z);
      cloud.points.push_back(point);
    }
    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(cloud, cloud_msg);
    cloud_msg.header.frame_id = "map";
    cloud_msg.header.stamp = this->now();
    centroids_pub_->publish(cloud_msg);
  }

  // Get robot current position (simplified)
  geometry_msgs::msg::Point start;
  start.x = 0.0;  // Assume robot at origin for demo
  start.y = 0.0;
  start.z = 0.0;

  // Plan path to first centroid using improved A*
  if (!centroids.empty()) {
    auto path = astar_path_planning(start, centroids[0], current_density_map_);
    path_pub_->publish(path);

    // Send navigation goal
    geometry_msgs::msg::PoseStamped goal;
    goal.header.frame_id = "map";
    goal.header.stamp = this->now();
    goal.pose.position = centroids[0];
    goal.pose.orientation.w = 1.0;
    send_navigation_goal(goal);
  }
}

std::vector<geometry_msgs::msg::Point> SemanticPathPlannerNode::kmeans_clustering(const std::vector<geometry_msgs::msg::Point>& points, int k)
{
  // Simple K-Means implementation
  std::vector<geometry_msgs::msg::Point> centroids(k);
  // Initialize centroids randomly
  for (int i = 0; i < k; ++i) {
    centroids[i] = points[i % points.size()];
  }

  // For simplicity, run fixed iterations
  for (int iter = 0; iter < 10; ++iter) {
    std::vector<std::vector<geometry_msgs::msg::Point>> clusters(k);
    for (const auto& p : points) {
      double min_dist = std::numeric_limits<double>::max();
      int best_cluster = 0;
      for (int c = 0; c < k; ++c) {
        double dist = std::hypot(p.x - centroids[c].x, p.y - centroids[c].y);
        if (dist < min_dist) {
          min_dist = dist;
          best_cluster = c;
        }
      }
      clusters[best_cluster].push_back(p);
    }

    // Update centroids
    for (int c = 0; c < k; ++c) {
      if (!clusters[c].empty()) {
        double sum_x = 0, sum_y = 0;
        for (const auto& p : clusters[c]) {
          sum_x += p.x;
          sum_y += p.y;
        }
        centroids[c].x = sum_x / clusters[c].size();
        centroids[c].y = sum_y / clusters[c].size();
      }
    }
  }

  return centroids;
}

double SemanticPathPlannerNode::potential_field_cost(int x, int y, const nav_msgs::msg::OccupancyGrid& grid)
{
  int index = y * grid.info.width + x;
  if (index < 0 || index >= grid.data.size()) return 100.0;
  return grid.data[index] / 100.0;  // Higher density, lower cost (attractive)
}

struct AStarNode {
  int x, y;
  double g, h, f;
  AStarNode* parent;

  AStarNode(int x, int y, double g, double h, AStarNode* parent)
    : x(x), y(y), g(g), h(h), f(g + h), parent(parent) {}

  bool operator>(const AStarNode& other) const {
    return f > other.f;
  }
};

nav_msgs::msg::Path SemanticPathPlannerNode::astar_path_planning(const geometry_msgs::msg::Point& start, const geometry_msgs::msg::Point& goal, const nav_msgs::msg::OccupancyGrid& grid)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  path.header.stamp = this->now();

  // Convert to grid coordinates
  int start_x = static_cast<int>((start.x - grid.info.origin.position.x) / grid.info.resolution);
  int start_y = static_cast<int>((start.y - grid.info.origin.position.y) / grid.info.resolution);
  int goal_x = static_cast<int>((goal.x - grid.info.origin.position.x) / grid.info.resolution);
  int goal_y = static_cast<int>((goal.y - grid.info.origin.position.y) / grid.info.resolution);

  // Check if start and goal are within bounds
  if (start_x < 0 || start_x >= grid.info.width || start_y < 0 || start_y >= grid.info.height ||
      goal_x < 0 || goal_x >= grid.info.width || goal_y < 0 || goal_y >= grid.info.height) {
    // Return empty path if out of bounds
    return path;
  }

  // A* algorithm
  std::priority_queue<AStarNode, std::vector<AStarNode>, std::greater<AStarNode>> open_list;
  std::vector<std::vector<bool>> closed_list(grid.info.height, std::vector<bool>(grid.info.width, false));
  std::vector<std::vector<double>> g_scores(grid.info.height, std::vector<double>(grid.info.width, std::numeric_limits<double>::max()));

  // Start node
  double h = std::hypot(goal_x - start_x, goal_y - start_y);
  open_list.emplace(start_x, start_y, 0.0, h, nullptr);
  g_scores[start_y][start_x] = 0.0;

  // Movement directions (8-directional)
  std::vector<std::pair<int, int>> directions = {{-1, -1}, {-1, 0}, {-1, 1}, {0, -1}, {0, 1}, {1, -1}, {1, 0}, {1, 1}};

  while (!open_list.empty()) {
    AStarNode current = open_list.top();
    open_list.pop();

    // Check if goal reached
    if (current.x == goal_x && current.y == goal_y) {
      // Reconstruct path
      std::vector<geometry_msgs::msg::Point> path_points;
      AStarNode* node = &current;
      while (node) {
        geometry_msgs::msg::Point p;
        p.x = grid.info.origin.position.x + node->x * grid.info.resolution + grid.info.resolution / 2;
        p.y = grid.info.origin.position.y + node->y * grid.info.resolution + grid.info.resolution / 2;
        p.z = 0.0;
        path_points.push_back(p);
        node = node->parent;
      }
      std::reverse(path_points.begin(), path_points.end());

      // Convert to Path message
      for (const auto& point : path_points) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = path.header;
        pose.pose.position = point;
        pose.pose.orientation.w = 1.0;
        path.poses.push_back(pose);
      }
      return path;
    }

    if (closed_list[current.y][current.x]) {
      continue;
    }
    closed_list[current.y][current.x] = true;

    // Explore neighbors
    for (const auto& dir : directions) {
      int neighbor_x = current.x + dir.first;
      int neighbor_y = current.y + dir.second;

      // Check bounds
      if (neighbor_x < 0 || neighbor_x >= grid.info.width || neighbor_y < 0 || neighbor_y >= grid.info.height) {
        continue;
      }

      if (closed_list[neighbor_y][neighbor_x]) {
        continue;
      }

      // Calculate cost
      double move_cost = dir.first != 0 && dir.second != 0 ? 1.414 : 1.0; // Diagonal cost
      int index = neighbor_y * grid.info.width + neighbor_x;
      double density_cost = grid.data[index] / 100.0; // Normalize density
      
      // Make higher density areas more attractive (lower cost)
      // Balance between distance cost and density attraction
      double density_attraction = 1.0 - (density_cost * 0.7); // Higher density = lower cost
      density_attraction = std::max(0.3, density_attraction); // Minimum cost to avoid excessive attraction
      
      double total_cost = current.g + move_cost * density_attraction;

      if (total_cost < g_scores[neighbor_y][neighbor_x]) {
        g_scores[neighbor_y][neighbor_x] = total_cost;
        double h = std::hypot(goal_x - neighbor_x, goal_y - neighbor_y);
        open_list.emplace(neighbor_x, neighbor_y, total_cost, h, new AStarNode(current.x, current.y, current.g, current.h, current.parent));
      }
    }
  }

  // If no path found, return straight line
  geometry_msgs::msg::PoseStamped pose;
  pose.header = path.header;
  pose.pose.position = start;
  pose.pose.orientation.w = 1.0;
  path.poses.push_back(pose);

  pose.pose.position = goal;
  path.poses.push_back(pose);

  return path;
}

void SemanticPathPlannerNode::send_navigation_goal(const geometry_msgs::msg::PoseStamped& goal)
{
  if (!nav_client_) {
    RCLCPP_ERROR(this->get_logger(), "Navigation client not initialized");
    return;
  }

  if (!nav_client_->wait_for_action_server(std::chrono::seconds(5))) {
    RCLCPP_ERROR(this->get_logger(), "NavigateToPose action server not available");
    return;
  }

  auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
  goal_msg.pose = goal;

  auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
  send_goal_options.result_callback =
    [this](const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult & result) {
      switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
          RCLCPP_INFO(this->get_logger(), "Navigation goal succeeded");
          break;
        case rclcpp_action::ResultCode::ABORTED:
          RCLCPP_ERROR(this->get_logger(), "Navigation goal aborted");
          break;
        case rclcpp_action::ResultCode::CANCELED:
          RCLCPP_WARN(this->get_logger(), "Navigation goal canceled");
          break;
        default:
          RCLCPP_ERROR(this->get_logger(), "Navigation goal failed with unknown code");
          break;
      }
    };

  send_goal_options.feedback_callback =
    [this](const std::shared_ptr<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>> /*handle*/,
           const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback) {
      RCLCPP_DEBUG(this->get_logger(), "Navigation feedback: distance remaining %.2f meters",
                   feedback->distance_remaining);
    };

  RCLCPP_INFO(this->get_logger(), "Sending navigation goal to (%.2f, %.2f)",
              goal.pose.position.x, goal.pose.position.y);

  nav_client_->async_send_goal(goal_msg, send_goal_options);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SemanticPathPlannerNode>());
  rclcpp::shutdown();
  return 0;
}