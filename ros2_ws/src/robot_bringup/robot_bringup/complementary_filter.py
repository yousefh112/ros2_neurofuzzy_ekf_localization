import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import message_filters
import tf2_ros

class ComplementaryFilter(Node):
    def __init__(self):
        super().__init__('complementary_filter_node')

        # alpha parameters
        self.alpha1 = 0.5  # Weight for EKF-1 (GPS-INS)
        self.alpha2 = 0.5  # Weight for EKF-2 (GPS-Odom)

        # Subscription to both EKF
        self.sub_ekf1 = message_filters.Subscriber(self, Odometry, '/odometry/filtered_1')
        self.sub_ekf2 = message_filters.Subscriber(self, Odometry, '/odometry/filtered_2')

        # Sincronization: we use ApproximateTimeSynchronizer for dealing with possible small time differences between the two EKF outputs
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.sub_ekf1, self.sub_ekf2], queue_size=10, slop=0.1
        )
        self.ts.registerCallback(self.fusion_callback)

        # Publisher for fused odometry
        self.odom_pub = self.create_publisher(Odometry, '/odometry/fused', 10)

        # Broadcaster for the TF (map -> base_link)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.get_logger().info("Complementary Filter node started!")

    def fusion_callback(self, msg1, msg2):
        # Implementation of Equation (2): [x, y]_fused = alpha1 * [x, y]_1 + alpha2 * [x, y]_2
        fused_x = (self.alpha1 * msg1.pose.pose.position.x) + (self.alpha2 * msg2.pose.pose.position.x)
        fused_y = (self.alpha1 * msg1.pose.pose.position.y) + (self.alpha2 * msg2.pose.pose.position.y)
        
        # For Z and orientation, we perform a simple average or use msg1
        fused_z = (msg1.pose.pose.position.z + msg2.pose.pose.position.z) / 2.0

        # Creation of final Odometry message
        fused_odom = Odometry()
        fused_odom.header.stamp = self.get_clock().now().to_msg()
        fused_odom.header.frame_id = 'map'
        fused_odom.child_frame_id = 'base_footprint'

        fused_odom.pose.pose.position.x = fused_x
        fused_odom.pose.pose.position.y = fused_y
        fused_odom.pose.pose.position.z = fused_z
        
        # Copy the orientation from one of the EKF (you could also implement a more complex fusion for orientation if needed)
        fused_odom.pose.pose.orientation = msg1.pose.pose.orientation

        self.odom_pub.publish(fused_odom)

        # TF publish for Rviz visualization
        t = TransformStamped()
        t.header.stamp = fused_odom.header.stamp
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'
        t.transform.translation.x = fused_x
        t.transform.translation.y = fused_y
        t.transform.translation.z = fused_z
        t.transform.rotation = fused_odom.pose.pose.orientation

        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = ComplementaryFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()