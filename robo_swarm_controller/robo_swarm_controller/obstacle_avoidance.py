import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import ReliabilityPolicy, QoSProfile

class ObstacleAvoidanceNode(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance_node')

        # 1. PUBLISHER: Send commands to the wheels
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

        # 2. SUBSCRIBER: Listen to the "Eyes" (Lidar)
        # QoS BEST_EFFORT: If we miss a packet, ignore it (don't buffer old data)
        self.subscriber_ = self.create_subscription(
            LaserScan, 
            '/scan', 
            self.scan_callback, 
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )
        self.get_logger().info('Obstacle Avoidance Node Started!')

    def scan_callback(self, msg):
        # 1. Filter invalid data (0.0 or infinite)
        valid_ranges = [r for r in msg.ranges if r > 0.0]
        
        # 2. Find closest object
        if not valid_ranges:
            min_distance = 10.0 
        else:
            min_distance = min(valid_ranges)

        cmd = Twist()

        # 3. Reflex Logic
        if min_distance < 0.5:
            self.get_logger().warn(f'🛑 STOP! Obstacle at {min_distance:.2f}m')
            cmd.linear.x = 0.0   # Stop
            cmd.angular.z = 0.5  # Turn Left
        else:
            cmd.linear.x = 0.2   # Drive Forward
            cmd.angular.z = 0.0

        self.publisher_.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publisher_.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()