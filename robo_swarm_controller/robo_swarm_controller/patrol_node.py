import rclpy
from rclpy.node import Node
# We need 'Twist' because that is the standard message type for moving robots
from geometry_msgs.msg import Twist 
import time

class PatrolNode(Node):
    def __init__(self):
        # 1. Initialize the Node with the name 'patrol_node'
        super().__init__('patrol_node')
        
        # 2. Create a Publisher
        #   - Type: Twist (Linear/Angular velocity)
        #   - Topic: 'cmd_vel' (The standard topic robots listen to for movement)
        #   - Queue Size: 10 (If the robot is busy, keep 10 commands in backup)
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # 3. Set the "Heartbeat" (Timer)
        #   - This calls 'self.timer_callback' automatically every 0.1 seconds
        #   - 0.1s = 10Hz (Standard for basic control loops)
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        # 4. State Machine Variables
        #   - We use simple states to switch behavior: "MOVE_FORWARD" vs "TURN"
        self.state = "MOVE_FORWARD" 
        self.start_time = time.time()
        
        self.get_logger().info('Patrol Node Started! Warning: Robot will move.')

    def timer_callback(self):
        """
        This function runs 10 times per second. 
        It decides purely based on TIME: 
        - If < 2 seconds passed: Drive Forward.
        - If > 2 seconds passed: Turn.
        """
        msg = Twist()

        # Calculate how long we have been in the current state
        current_time = time.time()
        elapsed = current_time - self.start_time

        # --- LOGIC: STATE 1 (FORWARD) ---
        if self.state == "MOVE_FORWARD":
            msg.linear.x = 0.2   # Drive forward at 0.2 meters/second
            msg.angular.z = 0.0  # Do not turn
            
            # Transition Condition: After 2 seconds, switch to TURN
            if elapsed > 2.0:
                self.state = "TURN"
                self.start_time = current_time
                self.get_logger().info('Switching to State: TURN')

        # --- LOGIC: STATE 2 (TURN) ---
        elif self.state == "TURN":
            msg.linear.x = 0.0   # Stop moving forward
            msg.angular.z = 0.5  # Rotate Left at 0.5 radians/second
            
            # Transition Condition: After 3 seconds of turning, switch to FORWARD
            if elapsed > 3.0:
                self.state = "MOVE_FORWARD"
                self.start_time = current_time
                self.get_logger().info('Switching to State: MOVE_FORWARD')

        # 5. Send the command to the robot
        self.publisher_.publish(msg)

def main(args=None):
    # Standard ROS 2 Boilerplate
    rclpy.init(args=args)       # Start ROS 2 communications
    node = PatrolNode()         # Create our node
    
    try:
        rclpy.spin(node)        # Keep the node alive and listening
    except KeyboardInterrupt:
        pass
    finally:
        # Safety: Stop the robot before shutting down!
        # If we don't do this, the robot might keep moving forever.
        stop_msg = Twist()
        node.publisher_.publish(stop_msg)
        
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()