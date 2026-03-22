import launch
import launch_ros.actions
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),

        launch_ros.actions.Node(
            package='turtlebot3_vision',
            executable='yolo_detector_node',
            name='yolo_detector_node',
            output='screen'
        ),

        launch_ros.actions.Node(
            package='semantic_density_map',
            executable='density_map_builder_node',
            name='density_map_builder_node',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }]
        ),

        launch_ros.actions.Node(
            package='semantic_density_map',
            executable='semantic_path_planner_node',
            name='semantic_path_planner_node',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }]
        ),
    ])