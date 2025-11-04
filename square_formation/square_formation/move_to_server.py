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
from square_formation_interfaces.action import MoveTo
import tf_transformations
from math import atan2, sqrt, pi
import time
import threading
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor


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

        self.odom_group = MutuallyExclusiveCallbackGroup()
        self.action_group = MutuallyExclusiveCallbackGroup()

        # Odometry subscription
        self.position = None
        self.yaw = 0.0
        self.odom_sub = self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            10,
            callback_group=self.odom_group
        )

        # Velocity publisher
        self.cmd_pub = self.create_publisher(Twist, cmd_topic, 10)

        # Action server
        self._action_server = ActionServer(
            self,
            MoveTo,
            action_name,
            execute_callback=self.execute_callback,
            callback_group=self.action_group
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
        rate = 10  # Hz
        dt = 1.0 / rate

        # === Phase 1: Rotate to face the goal ===
        while rclpy.ok():
            if self.position is None:
                time.sleep(0.1)
                continue

            # Handle cancellation
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info("Goal canceled during alignment")
                return MoveTo.Result()

            dx = goal.x - self.position.x
            dy = goal.y - self.position.y
            angle_to_goal = atan2(dy, dx)
            angle_diff = self.normalize_angle(angle_to_goal - self.yaw)

            # Stop condition for rotation
            if abs(angle_diff) < 0.05:  # ~3 degrees
                print("aligned_1")
                break

            # Rotate in place
            cmd = Twist()
            cmd.angular.z = max(min(1.5 * angle_diff, 0.6), -0.6)
            self.cmd_pub.publish(cmd)

            time.sleep(dt)

        # Stop rotation
        self.cmd_pub.publish(Twist())
        time.sleep(0.2)

        # Phase 2: Move toward goal while adjusting heading
        while rclpy.ok():
            if self.position is None:
                time.sleep(0.1)
                continue

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return MoveTo.Result()

            dx = goal.x - self.position.x
            dy = goal.y - self.position.y
            distance = sqrt(dx**2 + dy**2)
            angle_to_goal = atan2(dy, dx)
            angle_diff = self.normalize_angle(angle_to_goal - self.yaw)

            # Feedback
            feedback_msg.current_x = self.position.x
            feedback_msg.current_y = self.position.y
            feedback_msg.distance_to_goal = distance
            goal_handle.publish_feedback(feedback_msg)

            # Stop if close enough
            if distance < 0.05:
                break

            # Move forward with heading correction
            cmd = Twist()
            
            # Reduce linear speed if angle_diff is large
            if abs(angle_diff) > 0.2:  # ~11 degrees
                cmd.linear.x = 0.05  # small forward to avoid standing still
            else:
                cmd.linear.x = max(min(0.25 * distance, 0.5), 0.15)

            cmd.angular.z = max(min(1.5 * angle_diff, 0.6), -0.6)
            self.cmd_pub.publish(cmd)

            time.sleep(dt)

        # Stop movement
        self.cmd_pub.publish(Twist())
        time.sleep(0.3)

        # === Phase 3: Final orientation alignment ===
        while rclpy.ok():
            if self.position is None:
                time.sleep(0.1)
                continue

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info("Goal canceled during final alignment")
                return MoveTo.Result()

            angle_diff = self.normalize_angle(goal.yaw - self.yaw)
            if abs(angle_diff) < 0.05:
                break

            cmd = Twist()
            cmd.angular.z = max(min(1.5 * angle_diff, 0.5), -0.5)
            self.cmd_pub.publish(cmd)
            time.sleep(dt)

        # Stop final rotation
        self.cmd_pub.publish(Twist())

        # === Done ===
        result = MoveTo.Result()
        result.success = True
        result.message = "Goal reached and oriented"
        goal_handle.succeed()
        self.get_logger().info("Goal reached successfully")

        return result

    
    def normalize_angle(self,angle):
        while angle > pi:
            angle -= 2.0 * pi
        while angle < -pi:
            angle += 2.0 * pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    node = MoveToServer()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()

    node.get_logger().info("MoveToServer is ready and spinning in background.")
    
    # Keep the node alive (the action server runs internally)
    try:
        while rclpy.ok():
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()