"""
@file patrol_node.py
@description
Baseline patrol controller for Phase 1.

This node drives the robot in a simple square-like pattern
using time-based state transitions.

Purpose:
- Verify ROS 2 control pipeline
- Provide predictable motion
- Serve as a baseline (non-AI) controller
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time


class PatrolNode(Node):
    """
    Simple state-machine-based patrol controller.

    States:
    - MOVE_FORWARD
    - TURN

    This node does NOT use sensors.
    It is intentionally blind and deterministic.
    """

    def __init__(self):
        super().__init__("patrol_node")

        # Publisher for velocity commands
        self.cmd_pub = self.create_publisher(
            Twist,
            "/cmd_vel",
            10
        )

        # Timer controls the update rate (10 Hz)
        self.timer = self.create_timer(
            0.1,
            self.timer_callback
        )

        # State machine variables
        self.state = "MOVE_FORWARD"
        self.state_start_time = time.time()

        self.get_logger().info("Patrol node started (Phase 1 baseline).")

    def timer_callback(self):
        """
        Periodic control callback.

        Behavior:
        - Move forward for a fixed duration
        - Turn for a fixed duration
        - Repeat
        """

        msg = Twist()
        elapsed = time.time() - self.state_start_time

        if self.state == "MOVE_FORWARD":
            # Drive forward
            msg.linear.x = 0.2
            msg.angular.z = 0.0

            # After 2 seconds, switch to TURN
            if elapsed > 2.0:
                self.state = "TURN"
                self.state_start_time = time.time()

        elif self.state == "TURN":
            # Rotate in place
            msg.linear.x = 0.0
            msg.angular.z = 0.5

            # After 3 seconds, switch back to MOVE_FORWARD
            if elapsed > 3.0:
                self.state = "MOVE_FORWARD"
                self.state_start_time = time.time()

        # Publish velocity command
        self.cmd_pub.publish(msg)


def main(args=None):
    """
    ROS 2 entry point.
    """
    rclpy.init(args=args)
    node = PatrolNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Safety: stop the robot on shutdown
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
