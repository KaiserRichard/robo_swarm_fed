import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import message_filters
import pandas as pd
import os

class DataCollector(Node):
    def __init__(self):
        super().__init__('data_collector_node')

        # 1. Define the Better Path
        # We find the user's home, then build the path to ai_brain/data/
        home = os.path.expanduser('~')
        self.csv_path = os.path.join(home, 'ros2_ws/src/robo_swarm_fed/ai_brain/data/robot_data.csv')
        
        self.get_logger().info(f"📁 Data Collector Ready. Saving to: {self.csv_path}")

        # 2. Set up Subscribers (Same as before)
        self.odom_sub = message_filters.Subscriber(self, Odometry, '/odom')
        self.scan_sub = message_filters.Subscriber(self, LaserScan, '/scan')
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.odom_sub, self.scan_sub], 10, 0.1
        )
        self.ts.registerCallback(self.listener_callback)
        self.data_buffer = []

    def listener_callback(self, odom_msg, scan_msg):
        linear_x = odom_msg.twist.twist.linear.x
        angular_z = odom_msg.twist.twist.angular.z
        
        # Save limited Lidar ranges (only 360 points)
        lidar_ranges = list(scan_msg.ranges[:360])
        
        row = [linear_x, angular_z] + lidar_ranges
        self.data_buffer.append(row)

        # Save every 50 steps
        if len(self.data_buffer) >= 50:
            self.save_to_csv()

    def save_to_csv(self):
        columns = ['linear_x', 'angular_z'] + [f'lidar_{i}' for i in range(360)]
        df = pd.DataFrame(self.data_buffer, columns=columns)

        # Append if file exists, write new if not
        if not os.path.isfile(self.csv_path):
            df.to_csv(self.csv_path, index=False)
        else:
            df.to_csv(self.csv_path, mode='a', header=False, index=False)

        self.get_logger().info(f"💾 Saved {len(self.data_buffer)} points to ai_brain/data/")
        self.data_buffer = []

def main(args=None):
    rclpy.init(args=args)
    node = DataCollector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.save_to_csv()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
