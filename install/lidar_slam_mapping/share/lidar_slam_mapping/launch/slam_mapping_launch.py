from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Paths to configuration files
    lidar_config_file = PathJoinSubstitution([
        FindPackageShare('rslidar_sdk'), 'config', 'config.yaml'
    ])

    slam_toolbox_params_file = PathJoinSubstitution([
        FindPackageShare('lidar_slam_mapping'), 'config', 'slam_toolbox_params.yaml'
    ])

    pointcloud_to_laserscan_params_file = PathJoinSubstitution([
        FindPackageShare('lidar_slam_mapping'), 'config', 'pointcloud_to_laserscan_params.yaml'
    ])

    nav2_params_file = PathJoinSubstitution([
        FindPackageShare('lidar_slam_mapping'), 'config', 'nav2_params.yaml'
    ])

    rviz_config_file = '/opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz'

    return LaunchDescription([
        # LiDAR Node
        Node(
            package='rslidar_sdk',
            executable='rslidar_sdk_node',
            name='rslidar_node',
            output='screen',
            parameters=[{
                'config_file': lidar_config_file
            }],
        ),

        # PointCloud2 to LaserScan Conversion
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            output='screen',
            parameters=[pointcloud_to_laserscan_params_file],
            remappings=[
                ('/cloud_in', '/rslidar_points'),  # Input: PointCloud2
                ('/scan', '/scan'),  # Output: LaserScan for SLAM Toolbox
            ],
        ),

        # SLAM Toolbox Node
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',  # Synchronous SLAM mode
            name='slam_toolbox',
            output='screen',
            parameters=[slam_toolbox_params_file],
            remappings=[
                ('/scan', '/scan'),  # Input LaserScan
                ('/map', '/map'),    # Output Map
            ],
        ),

        # Static Transform: Base Link to LiDAR
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0.5', '0', '0', '0', 'base_link', 'rslidar'],
            name='static_base_link_to_rslidar',
        ),

        # Nav2 Navigation Launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('nav2_bringup'), 'launch', 'navigation_launch.py'
            ])),
            launch_arguments={
                'params_file': nav2_params_file,
                'use_sim_time': 'false'
            }.items(),
        ),

        # RViz2 Node for Visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen',
        ),
    ])

