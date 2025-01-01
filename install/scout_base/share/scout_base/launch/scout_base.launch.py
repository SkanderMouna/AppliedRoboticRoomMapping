import os
import launch
import launch_ros

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation clock if true')

    port_name_arg = DeclareLaunchArgument(
        'port_name', default_value='can0',
        description='CAN bus name, e.g. can0')

    odom_frame_arg = DeclareLaunchArgument(
        'odom_frame', default_value='odom',
        description='Odometry frame id')

    base_link_frame_arg = DeclareLaunchArgument(
        'base_frame', default_value='base_link',
        description='Base link frame id')

    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic_name', default_value='odom',
        description='Odometry topic name')

    is_scout_mini_arg = DeclareLaunchArgument(
        'is_scout_mini', default_value='false',
        description='Scout mini model')

    is_omni_wheel_arg = DeclareLaunchArgument(
        'is_omni_wheel', default_value='false',
        description='Scout mini omni-wheel model')

    auto_reconnect_arg = DeclareLaunchArgument(
        'auto_reconnect', default_value='true',
        description='Attempts to re-establish CAN command mode')

    simulated_robot_arg = DeclareLaunchArgument(
        'simulated_robot', default_value='false',
        description='Whether running with simulator')

    sim_control_rate_arg = DeclareLaunchArgument(
        'control_rate', default_value='50',
        description='Simulation control loop update rate')

    # Scout base node
    scout_base_node = launch_ros.actions.Node(
        package='scout_base',
        executable='scout_base_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'port_name': LaunchConfiguration('port_name'),
            'odom_frame': LaunchConfiguration('odom_frame'),
            'base_frame': LaunchConfiguration('base_frame'),
            'odom_topic_name': LaunchConfiguration('odom_topic_name'),
            'is_scout_mini': LaunchConfiguration('is_scout_mini'),
            'is_omni_wheel': LaunchConfiguration('is_omni_wheel'),
            'auto_reconnect': LaunchConfiguration('auto_reconnect'),
            'simulated_robot': LaunchConfiguration('simulated_robot'),
            'control_rate': LaunchConfiguration('control_rate'),

            # Added parameters for odometry covariance
            'pose_covariance': [
                0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.1, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.1, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.1, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.1
            ],
            'twist_covariance': [
                0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.1, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.1, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.1, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.1
            ],
        }]
    )

    return LaunchDescription([
        use_sim_time_arg,
        port_name_arg,
        odom_frame_arg,
        base_link_frame_arg,
        odom_topic_arg,
        is_scout_mini_arg,
        is_omni_wheel_arg,
        auto_reconnect_arg,
        simulated_robot_arg,
        sim_control_rate_arg,
        scout_base_node
    ])

