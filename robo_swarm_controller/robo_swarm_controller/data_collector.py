import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import csv
import os
import time

class DataCollectorNode(Node):
    def __init__(self):
        super().__init__('data_collector_node')
        
        # 1. Subscribers
        # Listen to the Lidar (The "Eyes")
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10)
        
        # Listen to the Velocity Commands (The "Action")
        self.cmd_sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_callback, 10)

        # 2. Memory
        self.latest_scan = None
        self.scan_received = False

        # 3. File Setup
        # Save CSV in the home directory so it's easy to find
        self.file_path = os.path.expanduser('~/robot_data.csv')
        self.init_csv()
        
        self.get_logger().info(f'📁 Data Collector Ready. Saving to: {self.file_path}')

    def init_csv(self):
        # Create file with headers if it doesn't exist
        if not os.path.exists(self.file_path):
            with open(self.file_path, 'w', newline='') as f:
                writer = csv.writer(f)
                # Header: linear_x, angular_z, range_0, range_1, ... range_359
                header = ['linear_x', 'angular_z'] + [f'r_{i}' for i in range(360)]
                writer.writerow(header)

    def scan_callback(self, msg):
        # Just store the latest scan in memory
        self.latest_scan = msg.ranges
        self.scan_received = True

    def cmd_callback(self, msg):
        # This is the TRIGGER. When we move, we record WHY we moved.
        if self.scan_received:
            self.save_data(msg, self.latest_scan)

    def save_data(self, twist, ranges):
        # 1. Process Data
        # Ensure we have exactly 360 points (resize if needed)
        # (Real lidar might give more/less, simple resize logic for now)
        data_row = [twist.linear.x, twist.angular.z]
        
        # Take first 360 points or pad with 0.0 if not enough
        processed_ranges = list(ranges)[:360] 
        if len(processed_ranges) < 360:
            processed_ranges += [0.0] * (360 - len(processed_ranges))
            
        data_row += processed_ranges

        # 2. Write to Disk
        with open(self.file_path, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(data_row)
            
        self.get_logger().info(f'💾 Saved Data Point: Speed={twist.linear.x:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = DataCollectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()