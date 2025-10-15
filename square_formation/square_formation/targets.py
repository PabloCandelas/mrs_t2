#PDCP Multi robot systems
# code to read the odom position of multiple robots and publish them thorugh a topic.
# The order to publish the motions consists as follows:
#       motion 1: 1->2  2->3  3->4  4->1 
#       motion 2: 1->4  4->3  3->2  2->1
# To run this code is needed
# + launch turtlebots 
# + use this line in the terminal: "ros2 run square_formation targets --ros-args -p namespaces:=aire1,agua2,tierra3,fuego4"
#   * replace the namespace to the used ones
# Last update: 15/oct/2025

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from math import sqrt, pi
import tf_transformations
from std_msgs.msg import String
import threading


from rclpy.parameter import Parameter

class Targets(Node):
    def __init__(self):
        super().__init__('Targets')

        # Declare parameter as string (not empty list)
        self.declare_parameter('namespaces', '')

        param_value = self.get_parameter('namespaces').get_parameter_value().string_value
        if not param_value:
            self.get_logger().error(
                "No namespaces provided! Please pass parameter 'namespaces'")
            rclpy.shutdown()
            return

        # Split comma-separated string into list
        self.namespaces = [s.strip() for s in param_value.split(',') if s.strip()]

        # Create subscriptions
        for ns in self.namespaces:
            topic = f'/{ns}/odom'
            self.create_subscription(
                Odometry, topic, lambda msg, ns=ns: self.odom_callback(msg, ns), 10)
            self.get_logger().info(f'Subscribed to {topic}')

        self.pos_pub = self.create_publisher(String, 'all_positions', 10)
        self.positions = {}
        self.yaws = {}

        self.get_logger().info(f'Targets node started for namespaces: {self.namespaces}')


    def odom_callback(self, msg, ns):
        self.positions[ns] = msg.pose.pose.position
        q = msg.pose.pose.orientation
        (_, _, yaw) = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.yaws[ns] = yaw


    def publisher(self, order=None):
        if not self.positions:
            self.get_logger().warn("No odometry data yet!")
            return

        # Default order: the same order as declared
        ns_order = self.namespaces

        # Apply custom order if provided
        if order is not None:
            try:
                ns_order = [self.namespaces[i] for i in order]
            except IndexError:
                self.get_logger().error("Invalid order index provided.")
                return

        # Build output string in the chosen order
        data_lines = []
        for ns in ns_order:
            if ns in self.positions:
                p = self.positions[ns]
                yaw = self.yaws.get(ns, 0.0)
                data_lines.append(f"{ns}: x={p.x:.2f}, y={p.y:.2f}, yaw={yaw:.2f}")
            else:
                data_lines.append(f"{ns}: no data yet")

        msg = String()
        msg.data = "\n".join(data_lines)
        self.pos_pub.publish(msg)
        self.get_logger().info(f"Published positions (order { [ns for ns in ns_order] }):\n{msg.data}")




def main(args=None):
    rclpy.init(args=args)
    node = Targets()

    #Bonjour
    node.get_logger().info('\n\n\tHola!! targets\n')

    node.get_logger().info('Waiting for all odometry...')
    while rclpy.ok() and not all(ns in node.positions for ns in node.namespaces):
        rclpy.spin_once(node)

    # Start spinning in background
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    node.get_logger().info('Ready for input: type "1" or "2"')

    while rclpy.ok():
        key = input("Press 1 or 2 to publish positions in a specific order, or 'q' to quit: ").strip()
        if key.lower() == 'q':
            break
        elif key == "1":
            node.publisher(order=[1, 2, 3, 0])
        elif key == "2":
            node.publisher(order=[0, 3, 2, 1])
        else:
            print("Invalid input. Use 1, 2, or q.")


    #BYE BYE BYE 
    node.get_logger().info('\n\n\tBYEEE targets\n')

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()