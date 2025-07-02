#!/usr/bin/env python3
"""
Angular Scaling Test Script
Use this to test and tune angular scaling parameters
"""

import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class AngularTestNode(Node):
    def __init__(self):
        super().__init__('angular_test_node')

        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )

        self.get_logger().info("Angular scaling test node initialized")

    def test_in_place_turns(self):
        """Test in-place turning at different speeds"""
        self.get_logger().info("Testing in-place turns...")

        test_angular_speeds = [0.5, 1.0, 1.5, 2.0]

        for angular_speed in test_angular_speeds:
            self.get_logger().info(
                f"Testing angular speed: {angular_speed} rad/s")

            # Right turn
            msg = Twist()
            msg.linear.x = 0.0
            msg.angular.z = angular_speed

            self.get_logger().info(
                f"Right turn at {angular_speed} rad/s for 2 seconds")
            for _ in range(20):  # 2 seconds at 10Hz
                self.cmd_vel_publisher.publish(msg)
                time.sleep(0.1)

            # Stop
            self.stop_robot()
            time.sleep(1.0)

            # Left turn
            msg.angular.z = -angular_speed
            self.get_logger().info(
                f"Left turn at {angular_speed} rad/s for 2 seconds")
            for _ in range(20):  # 2 seconds at 10Hz
                self.cmd_vel_publisher.publish(msg)
                time.sleep(0.1)

            # Stop
            self.stop_robot()
            time.sleep(2.0)

    def test_mixed_motion(self):
        """Test combined linear and angular motion"""
        self.get_logger().info("Testing mixed motion...")

        test_cases = [
            (0.1, 0.5),   # Slow forward with turn
            (0.2, 1.0),   # Medium forward with turn
            (0.1, 1.5),   # Slow forward with fast turn
        ]

        for linear, angular in test_cases:
            self.get_logger().info(
                f"Testing linear: {linear} m/s, angular: {angular} rad/s")

            msg = Twist()
            msg.linear.x = linear
            msg.angular.z = angular

            for _ in range(30):  # 3 seconds
                self.cmd_vel_publisher.publish(msg)
                time.sleep(0.1)

            self.stop_robot()
            time.sleep(2.0)

    def stop_robot(self):
        """Send stop command"""
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0

        for _ in range(5):  # Send stop for 0.5 seconds
            self.cmd_vel_publisher.publish(msg)
            time.sleep(0.1)

    def run_test_sequence(self):
        """Run the complete test sequence"""
        self.get_logger().info("Starting angular scaling test sequence")
        time.sleep(2.0)  # Wait for system to be ready

        self.test_in_place_turns()
        time.sleep(3.0)

        self.test_mixed_motion()

        self.get_logger().info("Test sequence completed")
        self.stop_robot()


def main(args=None):
    rclpy.init(args=args)

    test_node = AngularTestNode()

    try:
        test_node.run_test_sequence()
    except KeyboardInterrupt:
        test_node.get_logger().info("Test interrupted by user")
    finally:
        test_node.stop_robot()
        test_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
