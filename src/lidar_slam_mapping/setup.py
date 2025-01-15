from setuptools import setup, find_packages

package_name = 'lidar_slam_mapping'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(include=[package_name, f"{package_name}.*"]),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/slam_mapping_launch.py'
        ]),
        ('share/' + package_name + '/config', [
            'config/slam_toolbox_params.yaml',
            'config/pointcloud_to_laserscan_params.yaml',
            'config/nav2_params.yaml',
            'config/scout_mini.yaml',  
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student',
    maintainer_email='student@todo.todo',
    description='A package for SLAM and autonomous driving using LiDAR.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'lidar_min_distance = lidar_slam_mapping.lidar_min_distance:main',
    ],
}

)
