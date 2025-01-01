from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
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
        FindPackageShare('nav2_bringup'), 'params', 'nav2_params.yaml'
    ])
    rviz_config_file = '/opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz'

    # Path to Nav2 bringup launch file
    navigation_launch_file = PathJoinSubstitution([
        FindPackageShare('nav2_bringup'), 'launch', 'navigation_launch.py'
    ])

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
                ('/cloud_in', '/rslidar_points'),
                ('/scan', '/scan'),
            ],
        ),

        # SLAM Toolbox Node
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_toolbox_params_file],
            remappings=[
                ('/scan', '/scan'),
            ],
        ),

        # Autonomous Driving Node
        Node(
            package='lidar_slam_mapping',
            executable='lidar_obstacle_avoidance',
            name='lidar_obstacle_avoidance',
            output='screen',
        ),

        # Static Transforms
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            name='static_map_to_odom',
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
            name='static_odom_to_base_link',
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '1.0', '0', '0', '0', 'base_link', 'rslidar'],
            name='static_base_link_to_rslidar',
        ),

        # RViz2 for visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen',
        )
        # Include Navigation2 (Nav2) Launch
      #   IncludeLaunchDescription(
      #       PythonLaunchDescriptionSource(navigation_launch_file),
      #       launch_arguments={
      #           'map': '/ros2_ws/src/lidar_slam_mapping/maps/my_map.yaml',
       #          'use_sim_time': 'false',
         #        'params_file': nav2_params_file,
         #        'autostart': 'true'
         #    }.items(),
        # ),
    ])

