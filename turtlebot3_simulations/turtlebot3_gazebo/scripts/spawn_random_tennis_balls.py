#!/usr/bin/env python3

import argparse
import random
import sys

import rclpy
from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import Pose, Point
from std_msgs.msg import Float32MultiArray


def make_ball_sdf(radius=0.033, mass=0.058):
    # Simplified SDF for a tennis ball (visual + collision)
    return f"""<?xml version='1.0'?>
<sdf version='1.6'>
  <model name='tennis_ball'>
    <static>false</static>
    <link name='link'>
      <pose>0 0 {radius} 0 0 0</pose>
      <inertial>
        <mass>{mass}</mass>
        <inertia>
          <ixx>2.5e-5</ixx>
          <iyy>2.5e-5</iyy>
          <izz>2.5e-5</izz>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyz>0</iyz>
        </inertia>
      </inertial>
      <collision name='collision'>
        <geometry>
          <sphere><radius>{radius}</radius></sphere>
        </geometry>
      </collision>
      <visual name='visual'>
        <geometry>
          <sphere><radius>{radius}</radius></sphere>
        </geometry>
        <material>
          <script>
            <uri>file://media/materials/scripts/gazebo.material</uri>
            <name>Gazebo/Yellow</name>
          </script>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""


def main():
    parser = argparse.ArgumentParser(description='Spawn random tennis balls in Gazebo')
    parser.add_argument('--count', type=int, default=30, help='Number of balls to spawn')
    parser.add_argument('--xmin', type=float, default=-11.5)
    parser.add_argument('--xmax', type=float, default=11.5)
    parser.add_argument('--ymin', type=float, default=-5.0)
    parser.add_argument('--ymax', type=float, default=5.0)
    parser.add_argument('--seed', type=int, default=42, help='Random seed')
    args = parser.parse_args()

    rclpy.init(args=sys.argv)
    node = rclpy.create_node('spawn_random_tennis_balls')

    client = node.create_client(SpawnEntity, '/spawn_entity')
    if not client.wait_for_service(timeout_sec=10.0):
        node.get_logger().error('Service /spawn_entity not available')
        rclpy.shutdown()
        return

    # Publisher for ball positions
    ball_positions_pub = node.create_publisher(Float32MultiArray, 'spawned_ball_positions', 10)

    sdf = make_ball_sdf()
    ball_positions = []

    for i in range(args.count):
        name = f'tennis_ball_{i}'
        pose = Pose()
        pose.position.x = random.uniform(args.xmin, args.xmax)
        pose.position.y = random.uniform(args.ymin, args.ymax)
        pose.position.z = 0.033

        req = SpawnEntity.Request()
        req.name = name
        req.xml = sdf
        req.robot_namespace = ''
        req.initial_pose = pose

        future = client.call_async(req)
        rclpy.spin_until_future_complete(node, future)
        if future.result() is None:
            node.get_logger().error(f'Failed to spawn {name}: {future.exception()}')
        else:
            node.get_logger().info(f'Spawned {name} at ({pose.position.x:.2f}, {pose.position.y:.2f})')
            ball_positions.extend([pose.position.x, pose.position.y, pose.position.z])

    # Publish ball positions
    msg = Float32MultiArray()
    msg.data = ball_positions
    ball_positions_pub.publish(msg)
    node.get_logger().info(f'Published {len(ball_positions)//3} ball positions')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
