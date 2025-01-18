# README

## School Project: Autonomous Driving and Room Mapping Using Agilex Scout Mini

This project was developed as part of an applied robotics curriculum at Hof University. It utilizes the Agilex Scout Mini for autonomous driving, RS-LiDAR for room mapping, and a ZED camera for object recognition. The system is designed to run on Ubuntu 22.04 with ROS2 Humble.

---

### **Project Components**

#### **1. Agilex Scout Mini**

- **Repositories:**
  ```bash
  git clone https://github.com/westonrobot/ugv_sdk.git
  git clone https://github.com/westonrobot/scout_ros2.git
  ```
- **Setup and Testing:**
  1. Connect the Agilex Scout Mini via CAN bus.
  2. Initialize the CAN interface:
     ```bash
     sudo ip link set can0 up type can bitrate 500000
     ```
  3. Launch the Scout ROS2 driver:
     ```bash
     ros2 launch scout_base scout_base.launch.py
     ```
  4. Test teleoperation:
     ```bash
     ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p linear_speed:=0.01 -p angular_speed:=0.1
     ```

#### **2. RS-LiDAR for Room Mapping**

- **Repositories:**
  ```bash
  git clone https://github.com/RoboSense-LiDAR/rslidar_msg.git
  git clone https://github.com/RoboSense-LiDAR/rslidar_sdk.git
  git clone https://github.com/RoboSense-LiDAR/rs_driver.git
  ```
- **Setup:**
  1. Connect RS-LiDAR via Ethernet.
  2. Configure the RS-LiDAR IP to match the local network settings.
  3. Update the configuration file to use `RSHELIOS_16P`.
- **Launch and Visualize Point Cloud:**
  ```bash
  ros2 launch rslidar_sdk start.py
  ```
  Use RViz to visualize the 3D point cloud.

#### **3. SLAM Mapping**

- **Prerequisites:** Install `slam_toolbox` and `nav2`.
- **Configuration:**
  - Create YAML configuration files to convert 3D point clouds to 2D laser scans for improved SLAM performance.
- **Launch Mapping:**
  ```bash
  ros2 launch lidar_slam_mapping slam_mapping_launch.py
  ```
- **Save the Map:**
  ```bash
  ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/src/lidar_slam_mapping/maps/my_map.yaml
  ```

#### **4. Autonomous Driving**

- **Launch Autonomous Driving Node:**
  ```bash
  ros2 run lidar_slam_mapping lidar_min_distance
  ```

#### **5. Object Recognition with ZED Camera**

- **YOLO-Based Recognition:**
  - Developed a Python script: `zed2_yolo.py`.
  - Integrates ZED2 camera data for object detection using YOLO.

---

### **Workflow Summary**

1. **Setup Agilex Scout Mini:**

   - Connect via CAN and initialize the interface.
   - Launch the base driver for mobility control.

   ```bash
   sudo ip link set can0 up type can bitrate 500000
   ros2 launch scout_base scout_base.launch.py
   ```

2. **Launch Room Mapping:**

   - Connect RS-LiDAR via RJ45.
   - Start the SLAM mapping node:

   ```bash
   ros2 launch lidar_slam_mapping slam_mapping_launch.py
   ```

3. **Autonomous Driving:**

   - Run the autonomous driving node in parallel:

   ```bash
   ros2 run lidar_slam_mapping lidar_min_distance
   ```

4. **Save the Map:**

   - Once mapping is complete, save the map for future use:

   ```bash
   ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/src/lidar_slam_mapping/maps/my_map.yaml
   ```

5. **Object Recognition:**

   - Execute the YOLO-based object recognition script:

   ```bash
   python3 zed2_yolo.py
   ```

---

### **Project Highlights**

- **Platform:** Ubuntu 22.04 with ROS2 Humble.
- **Hardware:**
  - Agilex Scout Mini
  - RS-LiDAR (RSHELIOS\_16P)
  - ZED2 Camera
- **Software:**
  - SLAM Toolbox
  - Nav2 for navigation
  - YOLO for object recognition

---

### **Commands Overview**

1. **Agilex Setup:**

   ```bash
   sudo ip link set can0 up type can bitrate 500000
   ros2 launch scout_base scout_base.launch.py
   ```

2. **Room Mapping:**

   ```bash
   ros2 launch lidar_slam_mapping slam_mapping_launch.py
   ```

3. **Autonomous Driving:**

   ```bash
   ros2 run lidar_slam_mapping lidar_min_distance
   ```

4. **Save Map:**

   ```bash
   ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/src/lidar_slam_mapping/maps/my_map.yaml
   ```

5. **Object Recognition:**

   ```bash
   python3 zed2_yolo.py
   ```

---

### **Credits**

Developed by Hof University Students:
- Mouna Skander
- Amira Khider
- Alex Lewandowski
- Sezim Orozobkova
- Rassul Chsheriyazdanov

Utilizing repositories from Agilex, RoboSense, and YOLO for ROS2.

Github repo yolo weight cfg and names
https://pjreddie.com/media/files/yolov3.weights
https://github.com/pjreddie/darknet/blob/master/cfg/yolov3.cfg
https://github.com/pjreddie/darknet/blob/master/data/coco.names
https://pjreddie.com/media/files/yolov3-tiny.weights
https://github.com/pjreddie/darknet/blob/master/cfg/yolov3-tiny.cfg