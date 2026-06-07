
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import message_filters


class ComplementaryFilter(Node):
    def __init__(self):
        super().__init__('complementary_filter_node')

        
        self.alpha1 = 0.5   # Weight for KF-1 (GPS+IMU)
        self.alpha2 = 0.5   # Weight for KF-2 (GPS+Odom)

        # Subscriptions via ApproximateTimeSynchronizer
        self.sub_ekf1 = message_filters.Subscriber(self, Odometry, '/odometry/global')
        self.sub_ekf2 = message_filters.Subscriber(self, Odometry, '/odometry/global2')

        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.sub_ekf1, self.sub_ekf2],
            queue_size=10,
            slop=0.2,
        )
        self.ts.registerCallback(self.fusion_callback)

        # Output: [x_kf_c, y_kf_c] = training target for ANN (paper Eq. 3)
        self.odom_pub = self.create_publisher(Odometry, '/odometry/fused', 10)

        self.get_logger().info(
            '[ComplementaryFilter] Started. α1=0.5 (KF1) α2=0.5 (KF2)\n'
            '  In : /odometry/global  /odometry/global2\n'
            '  Out: /odometry/fused   (ANN training target, paper Eq.2)')

    def fusion_callback(self, msg1: Odometry, msg2: Odometry) -> None:
        """
        Fuse KF-1 and KF-2 position estimates.
        Paper Eq. (2): [x_kf_c, y_kf_c] = α1·[x_kf1,y_kf1] + α2·[x_kf2,y_kf2]
        """
        fused_x = self.alpha1 * msg1.pose.pose.position.x + \
                  self.alpha2 * msg2.pose.pose.position.x
        fused_y = self.alpha1 * msg1.pose.pose.position.y + \
                  self.alpha2 * msg2.pose.pose.position.y

        # Orientation: take from dominant weight source (here α1=α2, so use msg1)
        main_msg = msg1 if self.alpha1 >= self.alpha2 else msg2

        out = Odometry()
        out.header.stamp    = msg1.header.stamp
        out.header.frame_id = 'map'
        out.child_frame_id  = 'base_footprint' 

        out.pose.pose.position.x  = fused_x
        out.pose.pose.position.y  = fused_y
        out.pose.pose.position.z  = main_msg.pose.pose.position.z
        out.pose.pose.orientation = main_msg.pose.pose.orientation

        # Velocity: copy from dominant source for FLS slip-error computation
        out.twist.twist = main_msg.twist.twist

        # Covariance: copy from dominant source
        out.pose.covariance  = main_msg.pose.covariance
        out.twist.covariance = main_msg.twist.covariance

        self.odom_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ComplementaryFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()