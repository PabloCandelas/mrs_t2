#!/usr/bin/env python3
# Licensed under the Apache License, Version 2.0

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import sys, select, termios, tty, threading, time, random

MANUAL_KEYS = {
    '\x1b[A': (0.09, 0.0),    # up arrow
    '\x1b[B': (-0.06, 0.0),   # down arrow
    '\x1b[D': (0.0, 1.5),     # left arrow
    '\x1b[C': (0.0, -1.5),    # right arrow
}

class ReactifMotion(Node):
    def __init__(self):
        super().__init__('reactif_motion_no_think_jus_fwd')

        self.declare_parameter('namespace', '')
        self.robot_ns = self.get_parameter('namespace').get_parameter_value().string_value
        scan_topic = f'/{self.robot_ns}/scan' if self.robot_ns else '/scan'
        cmd_topic = f'/{self.robot_ns}/cmd_vel' if self.robot_ns else '/cmd_vel'

        # Motion parameters
        self.stop_dist = 0.22
        self.slow_dist = 0.38
        self.linear_speed = 0.09
        self.back_speed = 0.06
        self.angular_speed = 1.5

        # State machine
        self.state = 'IDLE'        # IDLE, MOVING_FORWARD, TURNING, BACKING, MANUAL
        self.turning_direction = 1  # 1=left, -1=right

        # Laser info
        self.front_min = float('inf')
        self.left_min = float('inf')
        self.right_min = float('inf')
        self.left_max = 0.0
        self.right_max = 0.0

        # Publishers/subscribers
        self.cmd_pub = self.create_publisher(Twist, cmd_topic, 10)
        self.create_subscription(LaserScan, scan_topic, self.scan_callback, 10)

        # Keyboard thread
        self.kb_thread = threading.Thread(target=self.keyboard_listener, daemon=True)
        self.kb_thread.start()

        # Timer for motion loop
        self.create_timer(0.1, self.motion_loop)

        self.get_logger().info(
            f"Started reactive node. Namespace: {self.robot_ns or '(none)'}\n"
            "Keyboard:\n"
            "  s - start autonomous\n"
            "  p - pause\n"
            "  m - manual mode toggle\n"
            "  q - quit\n"
            "Manual: arrow keys (↑ ↓ ← →)\n"
        )

    def scan_callback(self, msg: LaserScan):
        ranges = msg.ranges
        n = len(ranges)

        # Front ±30 degrees
        front = ranges[int(-30/360*n):] + ranges[:int(30/360*n)]
        left = ranges[int(240/360*n):int(300/360*n)]
        right = ranges[int(60/360*n):int(120/360*n)]

        clean = lambda r: [x for x in r if msg.range_min < x < msg.range_max]
        front = clean(front)
        left = clean(left)
        right = clean(right)

        self.front_min = min(front) if front else float('inf')
        self.left_min = min(left) if left else float('inf')
        self.right_min = min(right) if right else float('inf')
        self.left_max = max(left) if left else 0.0
        self.right_max = max(right) if right else 0.0

        # Periodic log
        now = time.time()
        if not hasattr(self, "_last_log_time"):
            self._last_log_time = 0.0
        if now - self._last_log_time > 1.0:
            self.get_logger().info(
                f"Front: {self.front_min:.2f} | Left min/max: {self.left_min:.2f}/{self.left_max:.2f} | "
                f"Right min/max: {self.right_min:.2f}/{self.right_max:.2f} | State: {self.state}"
            )
            self._last_log_time = now

    def keyboard_listener(self):
        settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        buf = ''
        try:
            while rclpy.ok():
                if select.select([sys.stdin], [], [], 0.05)[0]:
                    c = sys.stdin.read(1)
                    buf += c
                    if buf in MANUAL_KEYS and self.state == 'MANUAL':
                        lin, ang = MANUAL_KEYS[buf]
                        cmd = Twist()
                        cmd.linear.x = lin
                        cmd.angular.z = ang
                        self.cmd_pub.publish(cmd)
                        buf = ''
                    elif c in ['s','p','m','q']:
                        if c == 's':
                            self.state = 'MOVING_FORWARD'
                            self.get_logger().info("Autonomous motion started.")
                        elif c == 'p':
                            self.state = 'IDLE'
                            self.stop_robot()
                            self.get_logger().info("Paused.")
                        elif c == 'm':
                            self.state = 'MANUAL' if self.state != 'MANUAL' else 'IDLE'
                            self.get_logger().info(f"Manual mode {'ON' if self.state=='MANUAL' else 'OFF'}.")
                        elif c == 'q':
                            self.get_logger().info("Shutting down.")
                            rclpy.shutdown()
                            break
                        buf = ''
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

    def motion_loop(self):
        cmd = Twist()

        if self.state == 'MOVING_FORWARD':
            cmd.linear.x = self.linear_speed
            cmd.angular.z = 0.0

            # --- Proportional centering away from walls ---
            if self.front_min >= self.slow_dist:
                desired_side_dist = 0.5
                Kp = 1.0
                if self.left_min < desired_side_dist:
                    cmd.angular.z += Kp * (desired_side_dist - self.left_min)   # turn right
                if self.right_min < desired_side_dist:
                    cmd.angular.z -= Kp * (desired_side_dist - self.right_min)  # turn left

            # --- Reactive turn/back if obstacle too close ---
            if self.front_min < self.stop_dist:
                self.state = 'BACKING'
                self.turning_direction = self.choose_turn_direction()
                cmd.linear.x = -self.back_speed
                cmd.angular.z = self.angular_speed * self.turning_direction
            elif self.front_min < self.slow_dist:
                self.state = 'TURNING'
                self.turning_direction = self.choose_turn_direction()
                cmd.linear.x = 0.0
                cmd.angular.z = self.angular_speed * self.turning_direction

        elif self.state == 'BACKING':
            if self.front_min >= self.slow_dist:
                self.state = 'TURNING'
                cmd.linear.x = 0.0
                cmd.angular.z = self.angular_speed * self.turning_direction
            else:
                cmd.linear.x = -self.back_speed
                cmd.angular.z = self.angular_speed * self.turning_direction

        elif self.state == 'TURNING':
            if self.front_min >= self.slow_dist:
                self.state = 'MOVING_FORWARD'
                cmd.linear.x = self.linear_speed
                cmd.angular.z = 0.0
            else:
                cmd.linear.x = 0.0
                cmd.angular.z = self.angular_speed * self.turning_direction

        elif self.state in ['IDLE','MANUAL']:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        # --- Optional random wiggle to escape stuck situations ---
        if self.front_min < self.stop_dist and self.left_min < self.slow_dist and self.right_min < self.slow_dist:
            cmd.angular.z = self.angular_speed * random.choice([-1, 1])

        self.cmd_pub.publish(cmd)

    def choose_turn_direction(self):
        if self.left_max > self.right_max + 0.05:
            return 1  # left
        elif self.right_max > self.left_max + 0.05:
            return -1  # right
        else:
            return random.choice([-1, 1])

    def stop_robot(self):
        self.cmd_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = ReactifMotion()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
