import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist 
import time

class PatrolNode(Node):
    def __init__(self):
        super().__init__('patrol_node')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.state = "MOVE_FORWARD" 
        self.start_time = time.time()
        self.get_logger().info('Patrol Node Started!')

    def timer_callback(self):
        msg = Twist()
        elapsed = time.time() - self.start_time

        if self.state == "MOVE_FORWARD":
            msg.linear.x = 0.2
            msg.angular.z = 0.0
            if elapsed > 2.0:
                self.state = "TURN"
                self.start_time = time.time()

        elif self.state == "TURN":
            msg.linear.x = 0.0
            msg.angular.z = 0.5
            if elapsed > 3.0:
                self.state = "MOVE_FORWARD"
                self.start_time = time.time()

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = PatrolNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop_msg = Twist()
        node.publisher_.publish(stop_msg)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()