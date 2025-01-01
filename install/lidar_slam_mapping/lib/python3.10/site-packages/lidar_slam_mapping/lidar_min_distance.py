import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import math
from geometry_msgs.msg import Twist
import sys
import signal
from enum import Enum
import threading
import time




class Direction(Enum):
    LEFT = 1
    RIGHT = 2
    FORWARD = 3
    STOP = 4




class LidarObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('lidar_obstacle_avoidance')


        # Subscribe to LiDAR point cloud topic
        self.subscription = self.create_subscription(
            PointCloud2,
            '/scan',
            self.listener_callback,
            10)


        # Publish robot movement commands
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)


        # Parameters
        self.emergency_stop = False
        self.obstacle_threshold = 1.0  # Increased to 1m for earlier detection
        self.min_distance_threshold = 0.40  # Increased minimum distance
        self.forward_speed = 0.3
        self.rotation_speed = 0.4  # Increased rotation speed


        # Predictive parameters
        self.approach_slowdown_distance = 1.5  # Distance to start slowing down
        self.min_forward_speed = 0.1
        self.max_forward_speed = 0.3


        # Divide view into wider sectors (180 degrees total)
        self.sectors = {
            'front': (-25, 25),  # Narrowed front sector
            'front_left': (25, 90),
            'front_right': (-90, -25),
            'left': (90, 135),
            'right': (-135, -90)
        }


        # Shared state between threads
        self.current_direction = Direction.FORWARD
        self.lock = threading.Lock()
        self.latest_points = None
        self.points_lock = threading.Lock()
        self.current_speed = self.max_forward_speed


        # Setup emergency stop
        signal.signal(signal.SIGINT, self.signal_handler)


        # Start the obstacle detection thread
        self.obstacle_thread = threading.Thread(target=self.obstacle_detection_loop)
        self.obstacle_thread.daemon = True
        self.obstacle_thread.start()


        # Start the continuous movement thread
        self.movement_thread = threading.Thread(target=self.continuous_movement_loop)
        self.movement_thread.daemon = True
        self.movement_thread.start()


    def get_sector_distance(self, points, sector_angles):
        min_dist = float('inf')
        if points is None:
            return 999.0


        for point in points:
            try:
                x = point[0]
                y = point[1]
                z = point[2]


                # Filter out points that are too high or too low
                if abs(z) > 0.5:  # Adjust this value based on your robot's height
                    continue


                distance = math.sqrt(x ** 2 + y ** 2)
                angle = math.degrees(math.atan2(y, x))


                if sector_angles[0] <= angle <= sector_angles[1]:
                    min_dist = min(min_dist, distance)
            except (IndexError, TypeError, ValueError):
                continue


        return min_dist if min_dist != float('inf') else 999.0


    def analyze_surroundings(self, points):
        sector_distances = {}
        for sector, angles in self.sectors.items():
            sector_distances[sector] = self.get_sector_distance(points, angles)
        return sector_distances


    def listener_callback(self, msg):
        if self.emergency_stop:
            self.stop_robot()
            return


        try:
            point_list = []
            for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
                point_list.append(point)


            with self.points_lock:
                self.latest_points = point_list


        except Exception as e:
            self.get_logger().error(f'Error processing point cloud: {str(e)}')


    def obstacle_detection_loop(self):
        while not self.emergency_stop:
            with self.points_lock:
                points = self.latest_points


            if points is not None:
                sector_distances = self.analyze_surroundings(points)
                new_direction = self.choose_direction(sector_distances)


                with self.lock:
                    self.current_direction = new_direction
                    # Adjust speed based on front distance
                    front_distance = sector_distances['front']
                    self.adjust_speed(front_distance)


            time.sleep(0.01)  # Increased frequency to 10ms


    def adjust_speed(self, front_distance):
        if front_distance < self.obstacle_threshold:
            # Gradually reduce speed as we approach obstacles
            self.current_speed = max(
                self.min_forward_speed,
                self.max_forward_speed * (front_distance / self.approach_slowdown_distance)
            )
        else:
            self.current_speed = self.max_forward_speed


    def continuous_movement_loop(self):
        while not self.emergency_stop:
            with self.lock:
                direction = self.current_direction


            if direction == Direction.STOP:
                self.stop_robot()
            elif direction == Direction.FORWARD:
                self.move_forward()
            elif direction == Direction.LEFT:
                self.rotate_left()
            elif direction == Direction.RIGHT:
                self.rotate_right()


            time.sleep(0.01)  # Increased frequency to 10ms


    def choose_direction(self, sector_distances):
        try:
            # Emergency stop if too close in front
            if sector_distances['front'] < self.min_distance_threshold:
                return Direction.STOP


            # Check if front path is clear
            front_clear = sector_distances['front'] > self.obstacle_threshold


            if front_clear:
                return Direction.FORWARD


            # Find the direction with most clear space
            left_space = max(sector_distances['front_left'], sector_distances['left'])
            right_space = max(sector_distances['front_right'], sector_distances['right'])


            # Add hysteresis to prevent oscillation
            difference_threshold = 0.2  # 20cm difference required to change direction
            if abs(left_space - right_space) < difference_threshold:
                # Keep the current rotation direction if the difference is small
                if self.current_direction in [Direction.LEFT, Direction.RIGHT]:
                    return self.current_direction


            # Choose the direction with more space
            return Direction.LEFT if left_space > right_space else Direction.RIGHT


        except Exception as e:
            self.get_logger().error(f'Error choosing direction: {str(e)}')
            return Direction.STOP


    def move_forward(self):
        move_msg = Twist()
        move_msg.linear.x = self.current_speed
        move_msg.angular.z = 0.0
        self.publisher.publish(move_msg)


    def rotate_left(self):
        move_msg = Twist()
        move_msg.linear.x = 0.0
        move_msg.angular.z = self.rotation_speed
        self.publisher.publish(move_msg)


    def rotate_right(self):
        move_msg = Twist()
        move_msg.linear.x = 0.0
        move_msg.angular.z = -self.rotation_speed
        self.publisher.publish(move_msg)


    def stop_robot(self):
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        self.publisher.publish(stop_msg)


    def signal_handler(self, sig, frame):
        self.get_logger().info("Emergency stop triggered!")
        self.emergency_stop = True
        self.stop_robot()
        sys.exit(0)




def main(args=None):
    try:
        rclpy.init(args=args)
        node = LidarObstacleAvoidance()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutting down...")
        if 'node' in locals():
            node.stop_robot()
    except Exception as e:
        print(f"Error: {str(e)}")
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()




if __name__ == '__main__':
    main()

