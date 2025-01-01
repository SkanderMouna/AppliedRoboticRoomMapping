import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import math
from geometry_msgs.msg import Twist
import sys
import signal
from enum import Enum


class Direction(Enum):
    LEFT = 1
    RIGHT = 2
    FORWARD = 3
    STOP = 4


class LidarObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('lidar_obstacle_avoidance')  # Correct node name

        # Subscribe to LiDAR point cloud topic
        self.subscription = self.create_subscription(
            PointCloud2,
            '/rslidar_points',
            self.listener_callback,
            10
        )

        # Publish robot movement commands
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Parameters
        self.emergency_stop = False
        self.obstacle_threshold = 0.6  # 60cm threshold
        self.min_distance_threshold = 0.30  # 30cm minimum
        self.forward_speed = 0.1
        self.rotation_speed = 0.3

        # Divide front view into sectors
        self.sector_size = 30  # degrees
        self.sectors = {
            'front': (-30, 30),
            'front_left': (30, 60),
            'front_right': (-60, -30),
            'left': (60, 90),
            'right': (-90, -60)
        }

        # Navigation memory
        self.last_direction = Direction.FORWARD
        self.stuck_counter = 0
        self.rotation_time = 0

        # Setup emergency stop
        signal.signal(signal.SIGINT, self.signal_handler)

    def get_sector_distance(self, points, sector_angles):
        min_dist = float('inf')
        for point in points:
            # Safely extract x, y, z coordinates
            try:
                x = point[0]
                y = point[1]
                z = point[2]
            except (IndexError, TypeError):
                # If point doesn't have expected format, skip it
                continue

            # Calculate distance and angle
            try:
                distance = math.sqrt(x ** 2 + y ** 2)
                angle = math.degrees(math.atan2(y, x))

                # Check if angle is within sector bounds
                if sector_angles[0] <= angle <= sector_angles[1]:
                    min_dist = min(min_dist, distance)
            except (ValueError, TypeError):
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
            # Convert PointCloud2 to list of points
            point_list = []
            for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
                point_list.append(point)

            # Analyze surroundings
            sector_distances = self.analyze_surroundings(point_list)

            # Choose direction
            direction = self.choose_direction(sector_distances)

            # Execute movement
            if direction == Direction.STOP:
                self.stop_robot()
            elif direction == Direction.FORWARD:
                self.move_forward()
            elif direction == Direction.LEFT:
                self.rotate_left()
            elif direction == Direction.RIGHT:
                self.rotate_right()

            self.last_direction = direction

        except Exception as e:
            self.get_logger().error(f'Error processing point cloud: {str(e)}')
            self.stop_robot()

    def choose_direction(self, sector_distances):
        try:
            front_clear = sector_distances['front'] > self.obstacle_threshold
            left_clear = sector_distances['front_left'] > self.obstacle_threshold
            right_clear = sector_distances['front_right'] > self.obstacle_threshold

            # Emergency stop if too close in front
            if sector_distances['front'] < self.min_distance_threshold:
                return Direction.STOP

            # If front is clear, go forward
            if front_clear:
                self.stuck_counter = 0
                return Direction.FORWARD

            # Choose rotation direction based on clear space
            if left_clear and not right_clear:
                return Direction.LEFT
            elif right_clear and not left_clear:
                return Direction.RIGHT
            elif left_clear and right_clear:
                return Direction.LEFT if sector_distances['front_left'] > sector_distances[
                    'front_right'] else Direction.RIGHT

            # If stuck, alternate directions
            self.stuck_counter += 1
            if self.stuck_counter > 20:
                self.stuck_counter = 0
                return Direction.RIGHT if self.last_direction == Direction.LEFT else Direction.LEFT

            return Direction.LEFT

        except Exception as e:
            self.get_logger().error(f'Error choosing direction: {str(e)}')
            return Direction.STOP

    def move_forward(self):
        move_msg = Twist()
        move_msg.linear.x = self.forward_speed
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

