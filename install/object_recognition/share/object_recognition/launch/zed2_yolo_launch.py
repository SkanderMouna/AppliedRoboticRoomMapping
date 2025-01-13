from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='object_recognition',
            executable='zed2_yolo',
            name='zed2_yolo_node',
            output='screen',
            parameters=[{
                'confidence_threshold': 0.5,  # Example parameter
                'model_path': '/path/to/your/yolo/model',  # Adjust to your model location
            }]
        )
    ])
