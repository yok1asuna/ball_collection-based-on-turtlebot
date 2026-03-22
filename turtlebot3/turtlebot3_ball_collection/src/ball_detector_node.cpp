#include "ball_detector_node.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <std_msgs/msg/float32_multi_array.hpp>

BallDetectorNode::BallDetectorNode() : Node("ball_detector_node")
{
  ball_positions_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("ball_positions", 10);
  marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("ball_markers", 10);

  // Subscribe to spawned ball positions
  auto spawned_positions_sub = this->create_subscription<std_msgs::msg::Float32MultiArray>(
    "spawned_ball_positions", 10,
    [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
      simulated_balls_.clear();
      for (size_t i = 0; i < msg->data.size(); i += 3) {
        geometry_msgs::msg::Point point;
        point.x = msg->data[i];
        point.y = msg->data[i + 1];
        point.z = msg->data[i + 2];
        simulated_balls_.push_back(point);
      }
      RCLCPP_INFO(this->get_logger(), "Received %zu ball positions", simulated_balls_.size());
    });

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(1000),
    [this]() { timer_callback(); });
}

void BallDetectorNode::timer_callback()
{
  publish_ball_positions(simulated_balls_);
}

void BallDetectorNode::publish_ball_positions(const std::vector<geometry_msgs::msg::Point>& balls)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  visualization_msgs::msg::MarkerArray markers;

  for (size_t i = 0; i < balls.size(); ++i) {
    pcl::PointXYZ point(balls[i].x, balls[i].y, balls[i].z);
    cloud.points.push_back(point);

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = this->now();
    marker.ns = "balls";
    marker.id = i;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position = balls[i];
    marker.scale.x = marker.scale.y = marker.scale.z = 0.066;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    markers.markers.push_back(marker);
  }

  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);
  cloud_msg.header.frame_id = "map";
  cloud_msg.header.stamp = this->now();

  ball_positions_pub_->publish(cloud_msg);
  marker_pub_->publish(markers);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BallDetectorNode>());
  rclcpp::shutdown();
  return 0;
}
