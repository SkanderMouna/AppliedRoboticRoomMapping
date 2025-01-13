from setuptools import find_packages, setup

package_name = 'object_recognition'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        ('share/' + package_name + '/launch', [
            'launch/zed2_yolo_launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student',
    maintainer_email='skander.mouna@hotmail.com',
    description='Object recognition using ZED camera and YOLO.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'zed2_yolo = object_recognition.zed2_yolo:main',
        ],
    },
)
