import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import message_filters


class ComplementaryFilter(Node):
    def __init__(self):
        super().__init__('complementary_filter_node')

        # PESI FISSI (non più parametri ROS)
        self.alpha1 = 0.5  # Peso EKF-1
        self.alpha2 = 0.5  # Peso EKF-2

        # SOTTOSCRIZIONI
        self.sub_ekf1 = message_filters.Subscriber(
            self,
            Odometry,
            '/odometry/global'
        )  # EKF GPS-IMU

        self.sub_ekf2 = message_filters.Subscriber(
            self,
            Odometry,
            '/odometry/global2'
        )  # EKF GPS-odometry

        # SINCRONIZZAZIONE
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.sub_ekf1, self.sub_ekf2],
            queue_size=10,
            slop=0.2
        )
        self.ts.registerCallback(self.fusion_callback)

        # PUBLISHER
        self.odom_pub = self.create_publisher(
            Odometry,
            '/odometry/fused',
            10
        )

        self.get_logger().info(
            "Complementary Filter Node avviato con alpha fissi."
        )

    def fusion_callback(self, msg1, msg2):

        # FUSIONE POSIZIONE
        fused_x = (self.alpha1 * msg1.pose.pose.position.x) + \
                   (self.alpha2 * msg2.pose.pose.position.x)

        fused_y = (self.alpha1 * msg1.pose.pose.position.y) + \
                   (self.alpha2 * msg2.pose.pose.position.y)

        # Scelta del messaggio dominante per Z e orientamento
        main_msg = msg2 if self.alpha2 > self.alpha1 else msg1

        fused_z = main_msg.pose.pose.position.z
        fused_quat = main_msg.pose.pose.orientation

        # CREAZIONE OUTPUT
        fused_odom = Odometry()

        fused_odom.header.stamp = msg1.header.stamp
        fused_odom.header.frame_id = 'map'
        fused_odom.child_frame_id = 'base_link'

        fused_odom.pose.pose.position.x = fused_x
        fused_odom.pose.pose.position.y = fused_y
        fused_odom.pose.pose.position.z = fused_z
        fused_odom.pose.pose.orientation = fused_quat

        # Copia covarianza dal messaggio principale
        fused_odom.pose.covariance = main_msg.pose.covariance

        self.odom_pub.publish(fused_odom)


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