#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose2D
import re


class Coordinator(Node):
    def __init__(self):
        super().__init__('coordinator')

        # Subscribe to the 'all_positions' topic from the targets node
        self.sub = self.create_subscription(
            String, 'all_positions', self.positions_callback, 10)

        # Dictionary to store publishers per namespace
        self.publishers = {}

        self.get_logger().info('Coordinator node started — waiting for positions...')

    def positions_callback(self, msg):
        """
        Example message:
        source_ns:aire1, target_ns:agua2, x:0.51, y:0.54, yaw:0.21
        """
        text = msg.data.strip()
        if not text:
            self.get_logger().warn('Received empty /all_positions message')
            return

        # Find all "source_ns, target_ns, x, y, yaw" groups
        pattern = (
            r'source_ns:(\w+),\s*target_ns:(\w+),\s*x:([-0-9.]+),\s*y:([-0-9.]+),\s*yaw:([-0-9.]+)'
        )
        matches = re.findall(pattern, text)

        if not matches:
            self.get_logger().warn(f"Could not parse message:\n{text}")
            return

        for source_ns, target_ns, x_str, y_str, yaw_str in matches:
            x, y, yaw = float(x_str), float(y_str), float(yaw_str)

            topic_name = f'/{target_ns}/move_to_topic'

            # Create publisher if it doesn't exist yet
            if target_ns not in self.publishers:
                self.publishers[target_ns] = self.create_publisher(Pose2D, topic_name, 10)
                self.get_logger().info(f"Created publisher for {topic_name}")

            pose_msg = Pose2D()
            pose_msg.x = x
            pose_msg.y = y
            pose_msg.theta = yaw

            # Publish target position to the robot's namespace
            self.publishers[target_ns].publish(pose_msg)
            self.get_logger().info(
                f"Sent goal to {target_ns} → x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = Coordinator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
