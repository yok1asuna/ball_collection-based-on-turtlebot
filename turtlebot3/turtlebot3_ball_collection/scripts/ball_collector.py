#!/usr/bin/env python3

import math
import re
from typing import Optional

import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import DeleteEntity


class BallCollector(Node):
    def __init__(self) -> None:
        super().__init__('ball_collector')

        self.declare_parameter('model_states_topic', '/gazebo/model_states')
        self.declare_parameter('delete_service', '/delete_entity')
        self.declare_parameter('robot_model_name', 'turtlebot3_burger')
        self.declare_parameter('robot_name_fallback_substring', 'turtlebot3')
        self.declare_parameter('ball_name_regex', '(ball|tennis)')
        self.declare_parameter('collect_distance_m', 0.26)
        self.declare_parameter('collect_half_fov_rad', 0.52)
        self.declare_parameter('collect_cooldown_sec', 0.5)
        self.declare_parameter('check_period_sec', 0.1)

        self.model_states_topic = self.get_parameter('model_states_topic').get_parameter_value().string_value
        self.delete_service_name = self.get_parameter('delete_service').get_parameter_value().string_value
        self.robot_model_name = self.get_parameter('robot_model_name').get_parameter_value().string_value
        self.robot_name_fallback_substring = self.get_parameter('robot_name_fallback_substring').get_parameter_value().string_value
        self.ball_name_regex = self.get_parameter('ball_name_regex').get_parameter_value().string_value
        self.collect_distance_m = self.get_parameter('collect_distance_m').get_parameter_value().double_value
        self.collect_half_fov_rad = self.get_parameter('collect_half_fov_rad').get_parameter_value().double_value
        self.collect_cooldown_sec = self.get_parameter('collect_cooldown_sec').get_parameter_value().double_value
        self.check_period_sec = self.get_parameter('check_period_sec').get_parameter_value().double_value

        self.ball_pattern = re.compile(self.ball_name_regex, re.IGNORECASE)
        self.latest_model_states: Optional[ModelStates] = None
        self.delete_in_flight = False
        self.last_collect_time = self.get_clock().now()

        self.model_states_sub = self.create_subscription(
            ModelStates,
            self.model_states_topic,
            self.model_states_callback,
            10,
        )
        self.delete_client = self.create_client(DeleteEntity, self.delete_service_name)
        self.timer = self.create_timer(self.check_period_sec, self.try_collect_once)

        self.get_logger().info(
            f'BallCollector started: topic={self.model_states_topic}, delete_service={self.delete_service_name}, '
            f'distance={self.collect_distance_m:.2f}m, half_fov={self.collect_half_fov_rad:.2f}rad'
        )

    def model_states_callback(self, msg: ModelStates) -> None:
        self.latest_model_states = msg

    def try_collect_once(self) -> None:
        if self.delete_in_flight or self.latest_model_states is None:
            return

        elapsed = (self.get_clock().now() - self.last_collect_time).nanoseconds / 1e9
        if elapsed < self.collect_cooldown_sec:
            return

        msg = self.latest_model_states
        robot_idx = self.find_robot_index(msg)
        if robot_idx is None:
            self.get_logger().warn('Robot model not found in /gazebo/model_states', throttle_duration_sec=2.0)
            return

        robot_pose = msg.pose[robot_idx]
        robot_yaw = self.yaw_from_quaternion(
            robot_pose.orientation.x,
            robot_pose.orientation.y,
            robot_pose.orientation.z,
            robot_pose.orientation.w,
        )

        candidate_name = None
        candidate_distance = None

        for idx, name in enumerate(msg.name):
            if idx == robot_idx:
                continue
            if not self.ball_pattern.search(name):
                continue

            ball_pose = msg.pose[idx]
            dx = ball_pose.position.x - robot_pose.position.x
            dy = ball_pose.position.y - robot_pose.position.y
            distance = math.hypot(dx, dy)
            if distance > self.collect_distance_m:
                continue

            bearing = self.normalize_angle(math.atan2(dy, dx) - robot_yaw)
            if abs(bearing) > self.collect_half_fov_rad:
                continue

            if candidate_distance is None or distance < candidate_distance:
                candidate_name = name
                candidate_distance = distance

        if candidate_name is not None:
            self.delete_ball(candidate_name)

    def find_robot_index(self, msg: ModelStates) -> Optional[int]:
        for i, name in enumerate(msg.name):
            if name == self.robot_model_name:
                return i

        for i, name in enumerate(msg.name):
            if self.robot_name_fallback_substring and self.robot_name_fallback_substring in name:
                return i

        return None

    def delete_ball(self, model_name: str) -> None:
        if not self.delete_client.wait_for_service(timeout_sec=0.1):
            self.get_logger().warn(
                f'Delete service {self.delete_service_name} not available',
                throttle_duration_sec=2.0,
            )
            return

        req = DeleteEntity.Request()
        req.name = model_name

        self.delete_in_flight = True
        future = self.delete_client.call_async(req)
        future.add_done_callback(lambda f: self.on_delete_done(f, model_name))

    def on_delete_done(self, future, model_name: str) -> None:
        self.delete_in_flight = False
        self.last_collect_time = self.get_clock().now()

        try:
            resp = future.result()
            if resp is not None and resp.success:
                self.get_logger().info(f'Collected and removed model: {model_name}')
            else:
                status = '' if resp is None else resp.status_message
                self.get_logger().warn(f'Failed to remove {model_name}: {status}')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'DeleteEntity call failed for {model_name}: {exc}')

    @staticmethod
    def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def normalize_angle(angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BallCollector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
