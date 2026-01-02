"""
@file obstacle_avoidance.py
@description
Reactive obstacle avoidance controller for Phase 1.

This node uses LiDAR data to:
- Stop when an obstacle is too close
- Turn away to avoid collision

Acts as a SAFETY BASELINE and TEACHER
for later imitation learning.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class ObstacleAvoidanceNode(Node):
    """
    Simple reactive controller using LiDAR.

    Logic:
    - If closest obstacle < threshold → stop & turn
    - Else → move forward
    """

    def __init__(self):
        super().__init__("obstacle_avoidance_node")

        # Publisher for robot velocity
        self.cmd_pub = self.create_publisher(
            Twist,
            "/cmd_vel",
            10
        )

        # Subscriber for LiDAR data
        self.scan_sub = self.create_subscription(
            LaserScan,
            "/scan",
            self.scan_callback,
            10
        )

        # Safety threshold (meters)
        self.stop_distance = 0.5

        self.get_logger().info(
            "Obstacle avoidance node started (Phase 1 safety controller)."
        )

    def scan_callback(self, scan: LaserScan):
        """
        Process incoming LiDAR scan and publish velocity command.
        """

        # Filter invalid readings
        valid_ranges = [
            r for r in scan.ranges
            if r > 0.0
        ]

        # Default: no obstacle detected
        min_distance = min(valid_ranges) if valid_ranges else float("inf")

        cmd = Twist()

        if min_distance < self.stop_distance:
            # Obstacle too close → stop and turn
            cmd.linear.x = 0.0
            cmd.angular.z = 0.5
        else:
            # Path is clear → move forward
            cmd.linear.x = 0.2
            cmd.angular.z = 0.0

        self.cmd_pub.publish(cmd)


def main(args=None):
    """
    ROS 2 entry point.
    """
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop robot safely
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
