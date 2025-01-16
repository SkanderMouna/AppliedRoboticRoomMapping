import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math
from enum import Enum
import sys
import signal
from rclpy.qos import qos_profile_sensor_data


class Direction(Enum):
    LEFT = 1
    RIGHT = 2
    FORWARD = 3
    STOP = 4


class LidarObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('lidar_obstacle_avoidance')

        # Subscribe to LaserScan topic with SensorDataQoS
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            qos_profile_sensor_data  # Use SensorDataQoS for high-frequency data
        )

        # Publish robot movement commands
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Parameters
        self.emergency_stop = False
        self.obstacle_threshold = 0.7  # 70 cm threshold for normal obstacles
        self.min_distance_threshold = 0.30  # 30 cm minimum for emergency stop
        self.forward_speed = 0.6
        self.rotation_speed = 0.6

        # Define sectors (angles in degrees)
        self.sectors = {
            'front': (-30, 30),
            'front_left': (30, 90),
            'front_right': (-90, -30),
            'left': (90, 135),
            'right': (-135, -90)
        }

        # Navigation memory
        self.last_direction = Direction.FORWARD
        self.stuck_counter = 0
        self.rotation_time = 0

        # Setup emergency stop
        signal.signal(signal.SIGINT, self.signal_handler)

    def get_sector_distance(self, ranges, angle_min, angle_increment, sector_angles):
        """Calculate the minimum distance in a given sector."""
        min_dist = float('inf')

        for i, distance in enumerate(ranges):
            if math.isinf(distance) or math.isnan(distance) or distance < 0.01:
                continue

            angle_deg = math.degrees(angle_min + i * angle_increment)

            if sector_angles[0] <= angle_deg <= sector_angles[1]:
                min_dist = min(min_dist, distance)

        return min_dist if min_dist != float('inf') else 999.0

    def analyze_surroundings(self, ranges, angle_min, angle_increment):
        """Analyze distances in all defined sectors."""
        sector_distances = {}
        for sector, angles in self.sectors.items():
            sector_distances[sector] = self.get_sector_distance(ranges, angle_min, angle_increment, angles)
        return sector_distances

    def listener_callback(self, msg):
        """Callback for LaserScan messages."""
        if self.emergency_stop:
            self.stop_robot()
            return

        try:
            # Analyze LaserScan data
            sector_distances = self.analyze_surroundings(msg.ranges, msg.angle_min, msg.angle_increment)
            direction = self.choose_direction(sector_distances)

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
            self.get_logger().error(f'Error processing LaserScan: {str(e)}')
            self.stop_robot()

    def choose_direction(self, sector_distances):
        """Decide the robot's direction based on sector distances."""
        try:
            # Emergency stop if too close in front
            if sector_distances['front'] < self.min_distance_threshold:
                return Direction.STOP

            # Check if front path is clear
            front_clear = sector_distances['front'] > self.obstacle_threshold

            if front_clear:
                self.stuck_counter = 0
                return Direction.FORWARD

            # Find the direction with most clear space
            left_space = max(sector_distances['front_left'], sector_distances['left'])
            right_space = max(sector_distances['front_right'], sector_distances['right'])

            # Choose the direction with more space
            if left_space > right_space:
                return Direction.LEFT
            else:
                return Direction.RIGHT

        except Exception as e:
            self.get_logger().error(f'Error choosing direction: {str(e)}')
            return Direction.STOP

    def move_forward(self):
        """Move the robot forward."""
        move_msg = Twist()
        move_msg.linear.x = self.forward_speed
        move_msg.angular.z = 0.0
        self.publisher.publish(move_msg)

    def rotate_left(self):
        """Rotate the robot to the left."""
        move_msg = Twist()
        move_msg.linear.x = 0.0
        move_msg.angular.z = self.rotation_speed
        self.publisher.publish(move_msg)

    def rotate_right(self):
        """Rotate the robot to the right."""
        move_msg = Twist()
        move_msg.linear.x = 0.0
        move_msg.angular.z = -self.rotation_speed
        self.publisher.publish(move_msg)

    def stop_robot(self):
        """Stop the robot."""
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        self.publisher.publish(stop_msg)

    def signal_handler(self, sig, frame):
        """Handle emergency stop."""
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