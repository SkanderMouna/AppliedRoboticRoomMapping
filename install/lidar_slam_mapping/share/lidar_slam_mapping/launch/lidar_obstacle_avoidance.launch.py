from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lidar_slam_mapping',
            executable='lidar_obstacle_avoidance',
            name='lidar_obstacle_avoidance',
            output='screen',
        ),
    ])

