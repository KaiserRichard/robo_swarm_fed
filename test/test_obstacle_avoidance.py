import unittest
import rclpy
from robo_swarm_controller.obstacle_avoidance import ObstacleAvoidanceNode
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class TestObstacleAvoidanceNode(unittest.TestCase):
    
    @classmethod
    def setUpClass(cls):
        rclpy.init() # Initialize ROS 2 context
    
    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown() # Clean up ROS 2 context

    def setUp(self):
        self.node = ObstacleAvoidanceNode()

    def tearDown(self):
        self.node.destroy_node()

    def test_stop_reflex(self):
        scan = LaserScan()
        scan.ranges = [1.0, 1.0, 0.2, 1.0, 1.0]
        cmd = self.node.calculate_command(scan)
        print(f"\n[TEST] Input: {scan.ranges} -> Output Speed: {cmd.linear.x}")
        self.assertEqual(cmd.linear.x, 0.0)
        self.assertNotEqual(cmd.angular.z, 0.0)

    def test_clear_path(self):
        scan = LaserScan()
        scan.ranges = [1.0, 1.0, 1.0, 1.0, 1.0]
        cmd = self.node.calculate_command(scan)
        print(f"\n[TEST] Input: {scan.ranges} -> Output Speed: {cmd.linear.x}")
        self.assertGreater(cmd.linear.x, 0.0, "Robot should move forward when path is clear.")

if __name__ == '__main__':  
    unittest.main()