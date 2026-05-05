import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class ComplementaryFilter(Node):
    def __init__(self):
        super().__init__('complementary_filter_node')

        self.alpha1 = 0.5 
        self.alpha2 = 0.5 

        self.msg1 = None
        self.msg2 = None

        # Subscribers
        self.create_subscription(Odometry, '/odometry/filtered_1', self.cb1, 10)
        self.create_subscription(Odometry, '/odometry/filtered_2', self.cb2, 10)

        # Publisher (Topic only, NO TF broadcasting to avoid fighting Gazebo)
        self.odom_pub = self.create_publisher(Odometry, '/odometry/fused', 10)

        # High-speed loop (20Hz)
        self.timer = self.create_timer(0.05, self.core_loop)
        self.get_logger().info("--- MAP-LESS FUSION NODE STARTED ---")

    def cb1(self, msg):
        self.msg1 = msg

    def cb2(self, msg):
        self.msg2 = msg

    def core_loop(self):
        # PUBLISH FUSED ODOMETRY OR PRINT ERRORS
        if self.msg1 and self.msg2:
            fused_x = (self.alpha1 * self.msg1.pose.pose.position.x) + (self.alpha2 * self.msg2.pose.pose.position.x)
            fused_y = (self.alpha1 * self.msg1.pose.pose.position.y) + (self.alpha2 * self.msg2.pose.pose.position.y)
            fused_z = (self.msg1.pose.pose.position.z + self.msg2.pose.pose.position.z) / 2.0

            fused_odom = Odometry()
            fused_odom.header.stamp = self.get_clock().now().to_msg()
            
            # The absolute frame is now odom, and the robot is base_footprint
            fused_odom.header.frame_id = 'odom'
            fused_odom.child_frame_id = 'base_footprint'

            fused_odom.pose.pose.position.x = fused_x
            fused_odom.pose.pose.position.y = fused_y
            fused_odom.pose.pose.position.z = fused_z
            fused_odom.pose.pose.orientation = self.msg1.pose.pose.orientation

            self.odom_pub.publish(fused_odom)
        else:
            status1 = "OK" if self.msg1 else "DEAD"
            status2 = "OK" if self.msg2 else "DEAD"
            self.get_logger().warn(f"Waiting... EKF1: [{status1}] | EKF2: [{status2}]", throttle_duration_sec=2.0)

def main(args=None):
    rclpy.init(args=args)
    node = ComplementaryFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()