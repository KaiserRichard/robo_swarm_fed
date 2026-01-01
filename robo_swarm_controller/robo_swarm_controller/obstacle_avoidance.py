#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class ObstacleAvoidanceNode(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance_node')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.get_logger().info("⚠️ Safety Node Started (TDD Mode)")

    def scan_callback(self, msg):
        # 1. Get the logical command
        twist = self.calculate_command(msg)
        # 2. Publish it
        self.publisher_.publish(twist)

    def calculate_command(self, scan_msg):
        """
        PURE LOGIC: Takes data, returns decision.
        No ROS publishing happens here.
        """
        twist = Twist()

        # Filter out "infinity" or 0.0 errors
        valid_ranges = [r for r in scan_msg.ranges if r > 0.0]
        if not valid_ranges:
            return twist # No data, do nothing

        min_distance = min(valid_ranges)

        if min_distance < 0.5:
            # REFLEX: STOP AND TURN
            twist.linear.x = 0.0
            twist.angular.z = 0.5
        else:
            # CRUISE MODE
            twist.linear.x = 0.2
            twist.angular.z = 0.0

        return twist

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

