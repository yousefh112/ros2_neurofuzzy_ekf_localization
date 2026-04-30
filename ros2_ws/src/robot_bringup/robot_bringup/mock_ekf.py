import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math

class MockEKF(Node):
    def __init__(self):
        super().__init__('mock_ekf')
        self.pub1 = self.create_publisher(Odometry, '/odometry/filtered_1', 10)
        self.pub2 = self.create_publisher(Odometry, '/odometry/filtered_2', 10)
        self.timer = self.create_timer(0.1, self.timer_callback) # 10Hz
        
        self.counter = 0.0

    def timer_callback(self):
        self.counter += 0.1
        
        # --- Message for EKF 1 ---
        msg1 = Odometry()
        msg1.header.stamp = self.get_clock().now().to_msg()
        msg1.header.frame_id = 'map'
        
        # Simulate linear movement + small offset
        msg1.pose.pose.position.x = self.counter 
        msg1.pose.pose.position.y = 1.0
        msg1.pose.pose.orientation.w = 1.0 # Valid quaternion!
        
        # --- Message for EKF 2 ---
        msg2 = Odometry()
        msg2.header.stamp = msg1.header.stamp # Same stamp to facilitate CF
        msg2.header.frame_id = 'map'
        
        # Assume EKF 2 sees the robot slightly ahead (measurement error)
        msg2.pose.pose.position.x = self.counter + 0.5 
        msg2.pose.pose.position.y = 1.2
        msg2.pose.pose.orientation.w = 1.0

        self.pub1.publish(msg1)
        self.pub2.publish(msg2)
        
        self.get_logger().info(f'Published pose: EKF1={msg1.pose.pose.position.x}, EKF2={msg2.pose.pose.position.x}')

def main(args=None):
    rclpy.init(args=args)
    node = MockEKF()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()