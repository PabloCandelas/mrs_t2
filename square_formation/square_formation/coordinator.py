#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose2D, PointStamped
from tf2_ros import Buffer, TransformListener
import tf_transformations
import re


class Coordinator(Node):
    def __init__(self):
        super().__init__('coordinator')

        # TF buffer & listener to access existing transformations
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscribe to the positions topic
        self.sub = self.create_subscription(String, 'all_positions', self.positions_callback, 10)

        self.publishers = {}
        self.get_logger().info('Coordinator node started. Waiting for /tf transforms...')

    def positions_callback(self, msg):
        text = msg.data.strip()
        if not text:
            self.get_logger().warn('Received empty /all_positions message')
            return

        pattern = (
            r'source_ns:(\w+),\s*target_ns:(\w+),\s*x:([-0-9.]+),\s*y:([-0-9.]+),\s*yaw:([-0-9.]+)'
        )
        matches = re.findall(pattern, text)

        if not matches:
            self.get_logger().warn(f"Could not parse message:\n{text}")
            return

        for source_ns, target_ns, x_str, y_str, yaw_str in matches:
            x, y, yaw = float(x_str), float(y_str), float(yaw_str)

            # Build frame names
            source_frame = f"{source_ns}/odom"
            target_frame = f"{target_ns}/odom"

            try:
                # Try to get the transform from source to target
                transform = self.tf_buffer.lookup_transform(
                    target_frame,
                    source_frame,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.5)
                )

                # Apply transform to the source point
                transformed_x, transformed_y = self.apply_tf_transform(x, y, transform)

                # Apply yaw rotation from transform as well
                yaw_t = yaw + self.yaw_from_quaternion(transform.transform.rotation)

                self.publish_target(target_ns, transformed_x, transformed_y, yaw_t)

            except Exception as e:
                self.get_logger().warn(
                    f"TF lookup failed between {source_frame} and {target_frame}: {e}"
                )

    def apply_tf_transform(self, x, y, transform):
        """Apply 2D transformation (translation + rotation) from TF transform."""
        tx = transform.transform.translation.x
        ty = transform.transform.translation.y

        q = transform.transform.rotation
        _, _, yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])

        x_new = x * tf_transformations.cos(yaw) - y * tf_transformations.sin(yaw) + tx
        y_new = x * tf_transformations.sin(yaw) + y * tf_transformations.cos(yaw) + ty
        return x_new, y_new

    def yaw_from_quaternion(self, q):
        """Extract yaw (in radians) from quaternion."""
        _, _, yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        return yaw

    def publish_target(self, target_ns, x, y, yaw):
        """Publish the transformed target pose to the target robot."""
        topic_name = f'/{target_ns}/move_to_topic'

        if target_ns not in self.publishers:
            self.publishers[target_ns] = self.create_publisher(Pose2D, topic_name, 10)
            self.get_logger().info(f"Created publisher for {topic_name}")

        pose_msg = Pose2D()
        pose_msg.x = x
        pose_msg.y = y
        pose_msg.theta = yaw

        self.publishers[target_ns].publish(pose_msg)
        self.get_logger().info(
            f"Sent transformed goal to {target_ns}: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}"
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