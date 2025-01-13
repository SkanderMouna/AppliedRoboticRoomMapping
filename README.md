sudo ip link set can0 up type can bitrate 500000
ros2 launch rslidar_sdk start.py use_sim_time:=True
ros2 launch scout_base scout_base.launch.py use_sim_time:=True
ros2 run lidar_slam_mapping lidar_min_distance use_sim_time:=True 

# if this work try 
sudo ip link set can0 up type can bitrate 500000
ros2 launch scout_base scout_base.launch.py 
ros2 launch lidar_slam_mapping slam_mapping_launch.py
ros2 run lidar_slam_mapping lidar_min_distance

ros2 launch lidar_slam_mapping slam_and_driving_launch.py
ros2 launch lidar_slam_mapping  mapping_autoDriving.py use_sim_time:=True
source ~/ros2_ws/install/setup.bash

ros2 launch rslidar_sdk start.py
ros2 run autonomous_driving lidar_min_distance

ros2 launch robot_control robot_control_launch.py

ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p linear_speed:=0.01 -p angular_speed:=0.1

ros2 launch lidar_slam_mapping slam_mapping_launch.py

ros2 launch object_recognition zed2_yolo_launch.py

ros2 launch autonomous_driving auto_drive_launch.py

# Applied Robotic Room Mapping

## Overview
**Applied Robotic Room Mapping** is a school project focused on implementing autonomous driving and room mapping capabilities on the Agilex Scout Mini robot platform. The project integrates advanced robotics technologies, including lidar-based SLAM (Simultaneous Localization and Mapping) using Rslidar and object recognition using a camera coupled with YOLO (You Only Look Once) for object detection.

## Objectives
- Enable autonomous navigation for the Agilex Scout Mini robot.
- Perform real-time room mapping using Rslidar.
- Implement object recognition and detection with a camera and YOLO.
- Demonstrate seamless integration of SLAM and object recognition for a complete robotic room mapping solution.

## Features
- **Autonomous Driving**: Using lidar and camera sensors, the robot navigates the environment autonomously while avoiding obstacles.
- **Room Mapping**: Real-time 2D/3D mapping of the environment using Rslidar.
- **Object Recognition**: Identification and localization of objects using YOLO object detection algorithm.

## Technologies Used
- **Robot Platform**: Agilex Scout Mini
- **Lidar**: Rslidar for SLAM
- **Object Recognition**: YOLO (You Only Look Once)
- **Programming Framework**: ROS 2 (Robot Operating System)

## Project Structure
```
project_root/
├── src/
│   ├── lidar_slam_mapping/    # Lidar SLAM implementation
│   ├── objectRecognition/     # YOLO object recognition scripts
│   ├── scout_ros2/            # Agilex Scout Mini ROS 2 interface
│   ├── rslidar_sdk/           # Rslidar SDK for lidar data processing
│   ├── ugv_sdk/               # SDK for the unmanned ground vehicle
│   └── other/                 # Additional utilities and scripts
├── rviz/                      # RViz configurations for visualization
├── setup.py                   # Setup file for project dependencies
└── README.md                  # Project documentation
```

## Getting Started

### Prerequisites
- **Hardware Requirements**:
  - Agilex Scout Mini robot
  - Rslidar
  - Camera (compatible with YOLO)
- **Software Requirements**:
  - ROS 2 Humble (or compatible version)
  - Python 3.8+
  - YOLO pre-trained model files

### Installation
1. Clone the repository:
   ```bash
   git clone https://github.com/SkanderMouna/AppliedRoboticRoomMapping.git
   ```

2. Navigate to the project directory:
   ```bash
   cd AppliedRoboticRoomMapping
   ```

3. Install dependencies:
   ```bash
   rosdep install --from-paths src --ignore-src -r -y
   colcon build
   ```

4. Source the ROS 2 environment:
   ```bash
   source install/setup.bash
   ```

### Running the Project
1. Launch the SLAM node:
   ```bash
   ros2 launch lidar_slam_mapping slam_launch.py
   ```

2. Start the object recognition node:
   ```bash
   ros2 run objectRecognition zed2_yolo.py
   ```

3. Visualize the mapping and detection in RViz:
   ```bash
   rviz2 -d rviz/slam_visualization.rviz
   ```

## Contributing
Contributions are welcome! If you'd like to contribute, please fork the repository and submit a pull request with your changes.

## License
This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.

## Acknowledgments
- Agilex Robotics for providing the Scout Mini platform.
- Open source contributors for ROS 2, YOLO, and Rslidar SDK.

---

For more information, feel free to contact the project maintainers via [GitHub Issues](https://github.com/SkanderMouna/AppliedRoboticRoomMapping/issues).
