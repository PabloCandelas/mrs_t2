#PDCP Multi robot systems
# code to work as the action server of the move to action
#   The action server accepts different namespaces as input
#   THe action server read the odom of the robot
#   The action server uses a p controller to reach the given coordinates
# To run this code is needed
# + launch turtlebots 
# + use this line in the terminal: "ros2 run square_formation move_to_server --ros-args -p namespace:=aire1"
#   * replace the namespace to the used ones
# Last update: 15/oct/2025


#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from square_formation.action import MoveTo
import tf_transformations
from math import atan2, sqrt

class MoveToServer(Node):
    def __init__(self):
        super().__init__('move_to_server')

        # Declare namespace parameter
        self.declare_parameter('namespace', '')
        ns = self.get_parameter('namespace').get_parameter_value().string_value
        self.ns = ns  # save for later

        self.get_logger().info(f'MoveToServer running for namespace: "{self.ns or "default"}"')

        # Topics with namespace
        odom_topic = f'/{self.ns}/odom' if self.ns else '/odom'
        cmd_topic = f'/{self.ns}/cmd_vel' if self.ns else '/cmd_vel'
        action_name = f'/{self.ns}/move_to_goal' if self.ns else 'move_to_goal'

        # Odometry subscription
        self.position = None
        self.yaw = 0.0
        self.odom_sub = self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            10
        )

        # Velocity publisher
        self.cmd_pub = self.create_publisher(Twist, cmd_topic, 10)

        # Action server
        self._action_server = ActionServer(
            self,
            MoveTo,
            action_name,
            execute_callback=self.execute_callback
        )

    def odom_callback(self, msg):
        self.position = msg.pose.pose.position
        q = msg.pose.pose.orientation
        _, _, self.yaw = tf_transformations.euler_from_quaternion(
            [q.x, q.y, q.z, q.w]
        )

    def execute_callback(self, goal_handle):
        goal = goal_handle.request
        self.get_logger().info(f'Received goal: x={goal.x}, y={goal.y}, yaw={goal.yaw}')

        feedback_msg = MoveTo.Feedback()

        rate = self.create_rate(10)  # 10 Hz loop

        while rclpy.ok():
            if self.position is None:
                rate.sleep()
                continue

            # Compute distance and heading
            dx = goal.x - self.position.x
            dy = goal.y - self.position.y
            distance = sqrt(dx**2 + dy**2)
            angle_to_goal = atan2(dy, dx)

            # Feedback
            feedback_msg.current_x = self.position.x
            feedback_msg.current_y = self.position.y
            feedback_msg.distance_to_goal = distance
            goal_handle.publish_feedback(feedback_msg)

            if distance < 0.05:  # close enough
                break

            # Simple proportional controller
            cmd = Twist()
            cmd.linear.x = min(0.2 * distance, 0.3)  # max speed 0.3 m/s
            cmd.angular.z = 2.0 * (angle_to_goal - self.yaw)
            self.cmd_pub.publish(cmd)

            rate.sleep()

        # Stop the robot
        self.cmd_pub.publish(Twist())

        # Set result
        result = MoveTo.Result()
        result.success = True
        result.message = "Goal reached"
        goal_handle.succeed()
        return result


def main(args=None):
    rclpy.init(args=args)
    node = MoveToServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()