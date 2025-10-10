#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import sqrt, pi
import tf_transformations
import time

class CoordinatedSquare(Node):
    def __init__(self):
        super().__init__('coordinated_square_v1')

        # Read namespaces parameter (comma-separated list)
        self.declare_parameter('namespaces', '')
        ns_param = self.get_parameter('namespaces').get_parameter_value().string_value
        self.namespaces = [ns.strip() for ns in ns_param.split(',') if ns.strip()]

        if not self.namespaces:
            self.get_logger().warn("No namespaces provided. Node will not control any robots.")

        # Data for each robot
        self.robots = {}
        for ns in self.namespaces:
            self.robots[ns] = {
                'odom': None,
                'yaw': 0.0,
                'prev_motion_type': None,
                'cmd_pub': self.create_publisher(Twist, f'/{ns}/cmd_vel', 10),
                'odom_sub': self.create_subscription(
                    Odometry, f'/{ns}/odom', lambda msg, n=ns: self.odom_callback(msg, n), 10
                )
            }

        self.get_logger().info(f"CoordinatedSquareV1 node started for namespaces: {', '.join(self.namespaces)}")

    def odom_callback(self, msg, ns):
        self.robots[ns]['odom'] = msg.pose.pose.position
        q = msg.pose.pose.orientation
        (_, _, yaw) = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.robots[ns]['yaw'] = yaw

    def wait_for_odom(self):
        self.get_logger().info('Waiting for odometry from all robots...')
        while rclpy.ok():
            all_ready = True
            rclpy.spin_once(self)
            for ns in self.namespaces:
                if self.robots[ns]['odom'] is None:
                    all_ready = False
            if all_ready:
                break
            time.sleep(0.1)
        self.get_logger().info('All robots have odometry. Ready for input.')

    def rotate_all(self, angle_deg, motion_type, angular_speed=0.3):
        for ns in self.namespaces:
            robot = self.robots[ns]
            if robot['odom'] is None:
                continue

            angle = angle_deg
            # Double angle if previous motion type is different
            if robot['prev_motion_type'] is not None and robot['prev_motion_type'] != motion_type:
                angle *= 2

            start_yaw = robot['yaw']
            target_angle = angle * pi / 180.0

            twist = Twist()
            twist.angular.z = angular_speed if target_angle > 0 else -angular_speed

            total_rotated = 0.0
            last_yaw = robot['yaw']

            while rclpy.ok() and abs(total_rotated) < abs(target_angle):
                rclpy.spin_once(self)
                delta_yaw = robot['yaw'] - last_yaw
                if delta_yaw > pi:
                    delta_yaw -= 2 * pi
                elif delta_yaw < -pi:
                    delta_yaw += 2 * pi
                total_rotated += delta_yaw
                last_yaw = robot['yaw']
                robot['cmd_pub'].publish(twist)
                time.sleep(0.01)

            twist.angular.z = 0.0
            robot['cmd_pub'].publish(twist)
            robot['prev_motion_type'] = motion_type
            self.get_logger().info(f"[{ns}] Rotated {angle:.1f} degrees.")

    def move_straight_all(self, distance, speed=0.1):
        for ns in self.namespaces:
            robot = self.robots[ns]
            if robot['odom'] is None:
                continue

            start_x = robot['odom'].x
            start_y = robot['odom'].y
            twist = Twist()
            twist.linear.x = speed

            total_distance = 0.0
            while rclpy.ok() and total_distance < distance:
                rclpy.spin_once(self)
                dx = robot['odom'].x - start_x
                dy = robot['odom'].y - start_y
                total_distance = sqrt(dx**2 + dy**2)
                robot['cmd_pub'].publish(twist)
                time.sleep(0.01)

            twist.linear.x = 0.0
            robot['cmd_pub'].publish(twist)
            self.get_logger().info(f"[{ns}] Moved {distance:.2f} m forward.")

def main(args=None):
    rclpy.init(args=args)
    node = CoordinatedSquare()

    node.wait_for_odom()

    while rclpy.ok():
        key = input("Press motion 1 or 2: ").strip()
        if key == "1":
            node.rotate_all(90, motion_type="1")
            node.move_straight_all(0.5)
        elif key == "2":
            node.rotate_all(-90, motion_type="2")
            node.move_straight_all(0.5)
        else:
            print("Invalid input. Use 1 or 2.")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
