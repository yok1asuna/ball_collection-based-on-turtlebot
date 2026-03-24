#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ball_collection_dir = get_package_share_directory('turtlebot3_ball_collection')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(ball_collection_dir, 'param', 'ball_collection.yaml'),
            description='Full path to param file to load'
        ),

        Node(
            package='turtlebot3_vision',
            executable='yolo_detector_node',
            name='yolo_detector',
            output='screen'
        ),

        Node(
            package='turtlebot3_ball_collection',
            executable='density_map_builder_node',
            name='density_map_builder',
            output='screen'
        )
    ])

