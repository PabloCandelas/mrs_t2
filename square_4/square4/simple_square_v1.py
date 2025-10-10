#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import sqrt, pi
import tf_transformations

class SimpleSquareV1(Node):
    def __init__(self):
        super().__init__('simple_square_v1')

        # Get namespace
        self.declare_parameter('namespace', '')
        ns = self.get_parameter('namespace').get_parameter_value().string_value

        odom_topic = f'/{ns}/odom' if ns else 'odom'
        cmd_vel_topic = f'/{ns}/cmd_vel' if ns else 'cmd_vel'

        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.odom_sub = self.create_subscription(Odometry, odom_topic, self.odom_callback, 10)

        self.position = None
        self.yaw = 0.0
        self.prev_motion = None
        self.prev_motion_type = None  # track motion 1 or 2


        self.get_logger().info(f'SimpleSquareV1 node started for namespace: {ns or "default"}')

    def odom_callback(self, msg):
        self.position = msg.pose.pose.position
        q = msg.pose.pose.orientation
        (_, _, self.yaw) = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])

    def move_straight(self, distance, speed=0.1):
        if self.position is None:
            self.get_logger().warn("No odometry yet.")
            return

        start_x = self.position.x
        start_y = self.position.y
        twist = Twist()
        twist.linear.x = speed

        while rclpy.ok():
            rclpy.spin_once(self)
            dx = self.position.x - start_x
            dy = self.position.y - start_y
            dist = sqrt(dx**2 + dy**2)
            if dist >= distance:
                break
            self.cmd_pub.publish(twist)

        twist.linear.x = 0.0
        self.cmd_pub.publish(twist)
        self.get_logger().info(f"Moved {distance:.2f} m forward.")

    def rotate(self, angle_deg, motion_type, angular_speed=0.3):
        if self.position is None:
            self.get_logger().warn("No odometry yet.")
            return

        # Double angle if previous motion type is different
        if self.prev_motion_type is not None and self.prev_motion_type != motion_type:
            angle_deg *= 2

        start_yaw = self.yaw
        target_angle = angle_deg * pi / 180.0

        twist = Twist()
        twist.angular.z = angular_speed if target_angle > 0 else -angular_speed

        total_rotated = 0.0
        last_yaw = self.yaw

        while rclpy.ok() and abs(total_rotated) < abs(target_angle):
            rclpy.spin_once(self)
            delta_yaw = self.yaw - last_yaw
            if delta_yaw > pi:
                delta_yaw -= 2 * pi
            elif delta_yaw < -pi:
                delta_yaw += 2 * pi
            total_rotated += delta_yaw
            last_yaw = self.yaw
            self.cmd_pub.publish(twist)

        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)

        # Store the motion type
        self.prev_motion_type = motion_type

        self.get_logger().info(f"Rotated {angle_deg:.1f} degrees.")


def main(args=None):
    rclpy.init(args=args)
    node = SimpleSquareV1()

    # Wait for odometry
    node.get_logger().info('Waiting for odometry...')
    while rclpy.ok() and node.position is None:
        rclpy.spin_once(node)

    node.get_logger().info('Ready for input: type "1" or "2"')

    while rclpy.ok():
        key = input("Press motion 1 or 2: ").strip()
        if key == "1":
            node.rotate(90, motion_type="1")
            node.move_straight(0.5)
        elif key == "2":
            node.rotate(-90, motion_type="2")
            node.move_straight(0.5)
        else:
            print("Invalid input. Use 1 or 2.")

    node.destroy_node()
    rclpy.shutdown()

    



if __name__ == '__main__':
    main()