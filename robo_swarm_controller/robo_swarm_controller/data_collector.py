"""
@file data_collector.py
@description
Data recording node for Phase 1.

This node records:
- LiDAR scans (input)
- Velocity commands (output)

It produces the dataset used in Phase 2 AI training.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import csv
import os


class DataCollectorNode(Node):
    """
    Records (LiDAR → cmd_vel) pairs to CSV.

    Design:
    - LiDAR is stored as the latest observation
    - Each velocity command triggers one data record
    """

    def __init__(self):
        super().__init__("data_collector_node")

        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan,
            "/scan",
            self.scan_callback,
            10
        )

        self.cmd_sub = self.create_subscription(
            Twist,
            "/cmd_vel",
            self.cmd_callback,
            10
        )

        # Store latest LiDAR scan
        self.latest_scan = None

        # Output CSV file
        self.csv_path = os.path.expanduser(
            "~/robot_data.csv"
        )

        # Initialize CSV file
        self._init_csv()

        self.get_logger().info(
            f"Data collector started. Saving to {self.csv_path}"
        )

    def _init_csv(self):
        """
        Create CSV file with header if it does not exist.
        """
        if not os.path.exists(self.csv_path):
            with open(self.csv_path, "w", newline="") as f:
                writer = csv.writer(f)

                # Header:
                # [linear_x, angular_z, r_0, r_1, ..., r_359]
                header = (
                    ["linear_x", "angular_z"]
                    + [f"r_{i}" for i in range(360)]
                )
                writer.writerow(header)

    def scan_callback(self, scan: LaserScan):
        """
        Store the most recent LiDAR scan.
        """
        self.latest_scan = scan.ranges

    def cmd_callback(self, cmd: Twist):
        """
        Record a data sample whenever a velocity command is issued.
        """
        if self.latest_scan is None:
            return

        # Ensure fixed LiDAR size
        ranges = list(self.latest_scan)[:360]
        if len(ranges) < 360:
            ranges += [0.0] * (360 - len(ranges))

        row = [
            cmd.linear.x,
            cmd.angular.z,
            *ranges
        ]

        with open(self.csv_path, "a", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(row)


def main(args=None):
    """
    ROS 2 entry point.
    """
    rclpy.init(args=args)
    node = DataCollectorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
