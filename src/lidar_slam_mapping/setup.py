from setuptools import setup, find_packages

package_name = 'lidar_slam_mapping'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(include=[package_name, f"{package_name}.*"]),
    data_files=[
        # Package metadata
        ('share/' + package_name, ['package.xml']),
        # Launch files
        ('share/' + package_name + '/launch', [
            'launch/lidar_obstacle_avoidance.launch.py',
            'launch/slam_mapping_launch.py',
        ]),
        # Configuration files
        ('share/' + package_name + '/config', [
            'config/slam_toolbox_params.yaml',
            'config/pointcloud_to_laserscan_params.yaml',
            'config/nav2_params.yaml',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='Lidar-based obstacle avoidance and SLAM mapping',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lidar_obstacle_avoidance = lidar_slam_mapping.lidar_obstacle_avoidance:main',
            'lidar_min_distance = lidar_slam_mapping.lidar_min_distance:main',
        ],
    },
)

