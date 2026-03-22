#ifndef TURTLEBOT3_BALL_COLLECTION__BALL_DETECTOR_NODE_HPP_
#define TURTLEBOT3_BALL_COLLECTION__BALL_DETECTOR_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

class BallDetectorNode : public rclcpp::Node
{
public:
  BallDetectorNode();

private:
  void timer_callback();
  void publish_ball_positions(const std::vector<geometry_msgs::msg::Point>& balls);

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ball_positions_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  // Simulated ball positions,read from spawn script
  std::vector<geometry_msgs::msg::Point> simulated_balls_;
};

#endif