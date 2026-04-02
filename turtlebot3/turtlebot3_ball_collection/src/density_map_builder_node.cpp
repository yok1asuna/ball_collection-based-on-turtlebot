#include "density_map_builder_node.hpp"
#include <cmath>
#include <algorithm>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::chrono_literals;

DensityMapBuilderNode::DensityMapBuilderNode() : Node("density_map_builder_node")
{
  target_poses_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
    "/vision/target_poses", 10,
    std::bind(&DensityMapBuilderNode::target_poses_callback, this, std::placeholders::_1));

  density_map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("density_map", 10);
  marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/visualization/density_markers", 10);
  nav_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
    this, "/navigate_to_pose");
  spin_client_ = rclcpp_action::create_client<nav2_msgs::action::Spin>(this, "/spin");

  this->declare_parameter<double>("resolution", 0.1);
  this->declare_parameter<int>("width", 100);
  this->declare_parameter<int>("height", 100);
  this->declare_parameter<double>("origin_x", -5.0);
  this->declare_parameter<double>("origin_y", -5.0);
  this->declare_parameter<int>("density_increment", 10);
  this->declare_parameter<int>("max_density", 100);
  this->declare_parameter<double>("gaussian_sigma", 0.5);
  this->declare_parameter<double>("time_decay_factor", 0.95);
  this->declare_parameter<bool>("navigate_on_peak", true);
  this->declare_parameter<double>("goal_republish_distance", 0.2);
  this->declare_parameter<std::string>("spin_action_name", "/spin");
  this->declare_parameter<bool>("enable_startup_spin", true);
  this->declare_parameter<double>("startup_spin_delay_sec", 1.0);
  this->declare_parameter<double>("lost_target_timeout_sec", 3.0);
  this->declare_parameter<double>("spin_angle_rad", 2.0 * M_PI);
  this->declare_parameter<double>("spin_cooldown_sec", 1.0);

  this->get_parameter("resolution", resolution_);
  this->get_parameter("width", width_);
  this->get_parameter("height", height_);
  this->get_parameter("origin_x", origin_x_);
  this->get_parameter("origin_y", origin_y_);
  this->get_parameter("density_increment", density_increment_);
  this->get_parameter("max_density", max_density_);
  this->get_parameter("gaussian_sigma", gaussian_sigma_);
  this->get_parameter("time_decay_factor", time_decay_factor_);
  this->get_parameter("navigate_on_peak", navigate_on_peak_);
  this->get_parameter("goal_republish_distance", goal_republish_distance_);
  this->get_parameter("spin_action_name", spin_action_name_);
  this->get_parameter("enable_startup_spin", enable_startup_spin_);
  this->get_parameter("startup_spin_delay_sec", startup_spin_delay_sec_);
  this->get_parameter("lost_target_timeout_sec", lost_target_timeout_sec_);
  this->get_parameter("spin_angle_rad", spin_angle_rad_);
  this->get_parameter("spin_cooldown_sec", spin_cooldown_sec_);

  spin_client_ = rclcpp_action::create_client<nav2_msgs::action::Spin>(this, spin_action_name_);

  pause_decay_srv_ = this->create_service<std_srvs::srv::Trigger>(
    "/pause_decay",
    std::bind(&DensityMapBuilderNode::handle_pause_decay, this, std::placeholders::_1, std::placeholders::_2));
  resume_decay_srv_ = this->create_service<std_srvs::srv::Trigger>(
    "/resume_decay",
    std::bind(&DensityMapBuilderNode::handle_resume_decay, this, std::placeholders::_1, std::placeholders::_2));

  lost_target_timer_ = this->create_wall_timer(
    200ms, std::bind(&DensityMapBuilderNode::monitor_lost_targets, this));
  startup_spin_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(startup_spin_delay_sec_)),
    std::bind(&DensityMapBuilderNode::maybe_startup_spin, this));

  last_detection_time_ = this->now();
  last_spin_time_ = this->now() - rclcpp::Duration::from_seconds(spin_cooldown_sec_ + 1.0);

  // Initialize TF2 buffer and listener
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  density_grid_.header.frame_id = "map";
  density_grid_.info.resolution = resolution_;
  density_grid_.info.width = width_;
  density_grid_.info.height = height_;
  density_grid_.info.origin.position.x = origin_x_;
  density_grid_.info.origin.position.y = origin_y_;
  density_grid_.info.origin.position.z = 0.0;
  density_grid_.info.origin.orientation.w = 1.0;
  density_grid_.data.assign(width_ * height_, 0);

  RCLCPP_INFO(
    this->get_logger(),
    "Density map builder ready. startup_spin=%s, lost_target_timeout=%.2fs, spin_action=%s",
    enable_startup_spin_ ? "true" : "false", lost_target_timeout_sec_, spin_action_name_.c_str());
}

void DensityMapBuilderNode::target_poses_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
  std::vector<geometry_msgs::msg::Point> points;
  points.reserve(msg->poses.size());
  for (const auto &pose : msg->poses) {
    points.push_back(pose.position);
  }

  if (points.empty()) {
    build_density_map(points);
    publish_density_map();
    send_peak_navigation_goal();
    return;
  }

  last_detection_time_ = this->now();

  RCLCPP_INFO(this->get_logger(), "Received %zu balls from %s frame", points.size(), msg->header.frame_id.c_str());

  // Transform points to map frame using TF2
  std::vector<geometry_msgs::msg::Point> transformed_points;
  try {
    // Get transform from point cloud frame to map frame
    geometry_msgs::msg::TransformStamped transform_stamped;
    try {
      transform_stamped = tf_buffer_->lookupTransform(
        "map", msg->header.frame_id, msg->header.stamp, std::chrono::seconds(1));
    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "TF2 transform error: %s", ex.what());
      return; 
    }

    // Transform each point to map frame
    for (const auto& p : points) {
      geometry_msgs::msg::PointStamped point_in, point_out;
      point_in.header.frame_id = msg->header.frame_id;
      point_in.header.stamp = msg->header.stamp;
      point_in.point = p;

      tf2::doTransform(point_in, point_out, transform_stamped);
      transformed_points.push_back(point_out.point);
    }
  } catch (std::exception &ex) {
    RCLCPP_ERROR(this->get_logger(), "Error transforming points: %s", ex.what());
    transformed_points = points;
  }

  build_density_map(transformed_points);
  publish_density_map();
  send_peak_navigation_goal();
}

void DensityMapBuilderNode::build_density_map(const std::vector<geometry_msgs::msg::Point>& points)
{
  // Keep peaks persistent while tactical scan is running.
  if (!decay_paused_) {
    for (size_t i = 0; i < density_grid_.data.size(); ++i) {
      density_grid_.data[i] = static_cast<int>(density_grid_.data[i] * time_decay_factor_);
    }
  }

  // Apply Gaussian distribution for each point
  for (const auto& p : points) {
    // Calculate the radius for Gaussian spread (3 sigma rule)
    int spread_radius = static_cast<int>(gaussian_sigma_ / resolution_ * 3);
    
    // Calculate center cell
    int center_x = static_cast<int>((p.x - origin_x_) / resolution_);
    int center_y = static_cast<int>((p.y - origin_y_) / resolution_);
    
    // Spread density using 2D Gaussian distribution
    for (int dx = -spread_radius; dx <= spread_radius; ++dx) {
      for (int dy = -spread_radius; dy <= spread_radius; ++dy) {
        int x = center_x + dx;
        int y = center_y + dy;
        
        if (x >= 0 && x < width_ && y >= 0 && y < height_) {
          // Calculate distance from center
          double distance = std::sqrt(dx * dx + dy * dy) * resolution_;
          
          // Calculate Gaussian weight
          double weight = std::exp(-(distance * distance) / (2 * gaussian_sigma_ * gaussian_sigma_));
          
          // Apply density increment with Gaussian weight
          int index = y * width_ + x;
          int new_density = static_cast<int>(density_grid_.data[index] + density_increment_ * weight);
          density_grid_.data[index] = std::min(max_density_, new_density);
        }
      }
    }
  }
}

void DensityMapBuilderNode::publish_density_map()
{
  density_grid_.header.stamp = this->now();
  density_map_pub_->publish(density_grid_);

  // Calculate and log density map statistics
  int max_density = 0;
  int high_density_cells = 0;
  int total_density = 0;
  for (int density : density_grid_.data) {
    if (density > max_density) {
      max_density = density;
    }
    if (density > 50) { // High density threshold
      high_density_cells++;
    }
    total_density += density;
  }
  double avg_density = density_grid_.data.empty() ? 0 : static_cast<double>(total_density) / density_grid_.data.size();

  RCLCPP_DEBUG(this->get_logger(), "Density map stats: max=%d, high_density_cells=%d, avg=%.2f",
               max_density, high_density_cells, avg_density);

  // Publish markers for visualization
  visualization_msgs::msg::MarkerArray markers;
  for (int i = 0; i < width_; ++i) {
    for (int j = 0; j < height_; ++j) {
      int index = j * width_ + i;
      if (density_grid_.data[index] > 0) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->now();
        marker.ns = "density";
        marker.id = index;
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = origin_x_ + i * resolution_;
        marker.pose.position.y = origin_y_ + j * resolution_;
        marker.pose.position.z = 0.0;
        marker.scale.x = marker.scale.y = resolution_;
        marker.scale.z = 0.01;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = density_grid_.data[index] / 100.0;
        markers.markers.push_back(marker);
      }
    }
  }
  marker_pub_->publish(markers);
}

void DensityMapBuilderNode::send_peak_navigation_goal()
{
  if (!navigate_on_peak_) {
    return;
  }

  if (spin_in_progress_) {
    return;
  }

  auto max_iter = std::max_element(density_grid_.data.begin(), density_grid_.data.end());
  if (max_iter == density_grid_.data.end() || *max_iter <= 0) {
    return;
  }

  const int max_index = static_cast<int>(std::distance(density_grid_.data.begin(), max_iter));
  const int x = max_index % width_;
  const int y = max_index / width_;

  geometry_msgs::msg::Point goal_point;
  goal_point.x = origin_x_ + (static_cast<double>(x) + 0.5) * resolution_;
  goal_point.y = origin_y_ + (static_cast<double>(y) + 0.5) * resolution_;
  goal_point.z = 0.0;

  if (has_last_goal_) {
    const double dx = goal_point.x - last_goal_point_.x;
    const double dy = goal_point.y - last_goal_point_.y;
    if (std::hypot(dx, dy) < goal_republish_distance_) {
      return;
    }
  }

  if (!nav_client_->wait_for_action_server(std::chrono::milliseconds(200))) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
      "navigate_to_pose action server not available");
    return;
  }

  nav2_msgs::action::NavigateToPose::Goal goal;
  goal.pose.header.frame_id = "map";
  goal.pose.header.stamp = this->now();
  goal.pose.pose.position = goal_point;
  goal.pose.pose.orientation.w = 1.0;

  nav_client_->async_send_goal(goal);
  last_goal_point_ = goal_point;
  has_last_goal_ = true;

  RCLCPP_INFO(this->get_logger(), "Sent navigation goal to density peak (%.2f, %.2f)",
    goal_point.x, goal_point.y);
}

void DensityMapBuilderNode::trigger_spin(const std::string & reason)
{
  if (spin_in_progress_) {
    return;
  }

  const rclcpp::Time now = this->now();
  if ((now - last_spin_time_).seconds() < spin_cooldown_sec_) {
    return;
  }

  if (!spin_client_->wait_for_action_server(500ms)) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "Spin action server '%s' not available", spin_action_name_.c_str());
    return;
  }

  set_decay_paused(true, reason.c_str());
  spin_in_progress_ = true;
  last_spin_time_ = now;

  nav2_msgs::action::Spin::Goal goal;
  goal.target_yaw = spin_angle_rad_;
  goal.time_allowance = rclcpp::Duration::from_seconds(30.0);

  rclcpp_action::Client<nav2_msgs::action::Spin>::SendGoalOptions options;
  options.goal_response_callback =
    std::bind(&DensityMapBuilderNode::on_spin_goal_response, this, std::placeholders::_1);
  options.result_callback =
    std::bind(&DensityMapBuilderNode::on_spin_result, this, std::placeholders::_1);

  spin_client_->async_send_goal(goal, options);
  RCLCPP_INFO(this->get_logger(), "Triggered spin (reason=%s, angle=%.2f rad)", reason.c_str(), spin_angle_rad_);
}

void DensityMapBuilderNode::on_spin_goal_response(
  const rclcpp_action::ClientGoalHandle<nav2_msgs::action::Spin>::SharedPtr & goal_handle)
{
  if (!goal_handle) {
    spin_in_progress_ = false;
    set_decay_paused(false, "spin_goal_rejected");
    RCLCPP_WARN(this->get_logger(), "Spin goal was rejected by Nav2");
    return;
  }
  RCLCPP_INFO(this->get_logger(), "Spin goal accepted");
}

void DensityMapBuilderNode::on_spin_result(
  const rclcpp_action::ClientGoalHandle<nav2_msgs::action::Spin>::WrappedResult & result)
{
  spin_in_progress_ = false;
  // Reset lost-target watchdog after each scan attempt to avoid immediate retrigger loops.
  last_detection_time_ = this->now();
  set_decay_paused(false, "spin_result");

  if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
    RCLCPP_INFO(this->get_logger(), "Spin completed successfully");
  } else if (result.code == rclcpp_action::ResultCode::ABORTED) {
    RCLCPP_WARN(this->get_logger(), "Spin aborted");
  } else if (result.code == rclcpp_action::ResultCode::CANCELED) {
    RCLCPP_WARN(this->get_logger(), "Spin canceled");
  } else {
    RCLCPP_WARN(this->get_logger(), "Spin ended with unknown result code");
  }

  // Immediately transition from scan phase back to peak-driven navigation.
  send_peak_navigation_goal();
}

void DensityMapBuilderNode::monitor_lost_targets()
{
  if (spin_in_progress_) {
    return;
  }

  const double seconds_without_target = (this->now() - last_detection_time_).seconds();
  if (seconds_without_target >= lost_target_timeout_sec_) {
    trigger_spin("lost_target");
    // Prevent immediate retrigger while still no detections.
    last_detection_time_ = this->now();
  }
}

void DensityMapBuilderNode::maybe_startup_spin()
{
  if (startup_spin_triggered_) {
    return;
  }

  startup_spin_triggered_ = true;
  startup_spin_timer_->cancel();
  if (enable_startup_spin_) {
    trigger_spin("startup");
  }
}

bool DensityMapBuilderNode::set_decay_paused(bool paused, const char * source)
{
  if (decay_paused_ == paused) {
    return false;
  }

  decay_paused_ = paused;
  RCLCPP_INFO(
    this->get_logger(),
    "Decay %s by %s",
    decay_paused_ ? "paused" : "resumed",
    source == nullptr ? "unknown" : source);
  return true;
}

void DensityMapBuilderNode::handle_pause_decay(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request;
  set_decay_paused(true, "service_pause_decay");
  response->success = true;
  response->message = "Decay paused";
}

void DensityMapBuilderNode::handle_resume_decay(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request;
  set_decay_paused(false, "service_resume_decay");
  response->success = true;
  response->message = "Decay resumed";
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DensityMapBuilderNode>());
  rclcpp::shutdown();
  return 0;
}