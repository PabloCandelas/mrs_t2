#PDCP Multi robot systems
# code to work as the action client of the move to action
#   The action client accepts different namespaces as input
#   THe action client gets the goal from a topic called '/move_to_topic'
#   The action client gets the feedback and all that fro mthe server and tells when the action is done
#   The action client doenst destroy after one use, it keeps waiting for the next goal
# To run this code is needed
# + launch turtlebots 
# + action server running 
# + use this line in the terminal: "ros2 run square_formation move_to_client --ros-args -p namespace:=aire1"
# + receive a goal from a topic for example: "ros2 topic pub /aire1/move_to_topic geometry_msgs/Pose2D "{x: 1.0, y: 0.5, theta: 0.0}""
#   * replace the namespace to the used ones
# Last update: 15/oct/2025

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Pose2D
from square_formation.action import MoveTo

class MoveToClient(Node):
    def __init__(self):
        super().__init__('move_to_client')

        # Namespace parameter
        self.declare_parameter('namespace', '')
        ns = self.get_parameter('namespace').get_parameter_value().string_value
        self.ns = ns

        # Namespaced action client
        action_name = f'/{self.ns}/move_to_goal' if self.ns else 'move_to_goal'
        self._client = ActionClient(self, MoveTo, action_name)
        self.get_logger().info(f'MoveToClient initialized for namespace: "{self.ns or "default"}"')

        # Subscriber for goals
        topic_name = f'/{self.ns}/move_to_topic' if self.ns else '/move_to_topic'
        self._sub = self.create_subscription(
            Pose2D,
            topic_name,
            self.goal_callback,
            10
        )
        self.get_logger().info(f'Subscribed to topic: "{topic_name}" for receiving goals')

    def goal_callback(self, msg: Pose2D):
        """Callback triggered when a new goal is received."""
        x_goal = msg.x
        y_goal = msg.y
        yaw_goal = msg.theta

        self.get_logger().info(f'Received goal from topic: x={x_goal}, y={y_goal}, yaw={yaw_goal}')
        self.send_goal(x_goal, y_goal, yaw_goal)

    def send_goal(self, x, y, yaw):
        """Send a MoveTo goal to the server."""
        self._client.wait_for_server()

        goal_msg = MoveTo.Goal()
        goal_msg.x = x
        goal_msg.y = y
        goal_msg.yaw = yaw

        self.get_logger().info(f'Sending goal: x={x}, y={y}, yaw={yaw}')
        self._send_goal_future = self._client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'Feedback: current_x={feedback.current_x:.2f}, '
            f'current_y={feedback.current_y:.2f}, '
            f'distance={feedback.distance_to_goal:.2f}'
        )

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: success={result.success}, message="{result.message}"')


def main(args=None):
    rclpy.init(args=args)
    node = MoveToClient()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Action client interrupted by user')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()