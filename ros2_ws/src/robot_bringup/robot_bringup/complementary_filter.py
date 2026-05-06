import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import message_filters
import tf2_ros

class ComplementaryFilter(Node):
    def __init__(self):
        super().__init__('complementary_filter_node')

        # 1. DECLARATION OF ROS 2 PARAMETERS
        # These parameters can be modified externally by the Behavior Tree
        self.declare_parameter('alpha1', 0.5)  # Weight EKF-1 (GPS)
        self.declare_parameter('alpha2', 0.5)  # Weight EKF-2 (GPS)
        self.declare_parameter('alpha3', 0.0)  # Weight EKF-3 (No-GPS / NN)

        # 2. SUBSCRIPTIONS TO THE THREE FILTERS
        # Make sure the topic names match your EKF nodes
        self.sub_ekf1 = message_filters.Subscriber(self, Odometry, '/odometry/filtered_1')
        self.sub_ekf2 = message_filters.Subscriber(self, Odometry, '/odometry/filtered_2')
        self.sub_ekf3 = message_filters.Subscriber(self, Odometry, '/odometry/filtered_3')

        # 3. SYNCHRONIZATION
        # We use ApproximateTimeSynchronizer to merge messages from the three filters
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.sub_ekf1, self.sub_ekf2, self.sub_ekf3], 
            queue_size=10, 
            slop=0.1
        )
        self.ts.registerCallback(self.fusion_callback)

        # 4. PUBLISHER AND TF
        self.odom_pub = self.create_publisher(Odometry, '/odometry/fused', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.get_logger().info("Complementary Filter Node started with parameter support!")

    def fusion_callback(self, msg1, msg2, msg3):
        # Dynamic parameter retrieval (updated by the Behavior Tree)
        a1 = self.get_parameter('alpha1').get_parameter_value().double_value
        a2 = self.get_parameter('alpha2').get_parameter_value().double_value
        a3 = self.get_parameter('alpha3').get_parameter_value().double_value

        # Fusion logic: implementation of the weighted equation
        # x_final = a1*P1(x) + a2*P2(x) + a3*P3(x)
        fused_x = (a1 * msg1.pose.pose.position.x) + \
                  (a2 * msg2.pose.pose.position.x) + \
                  (a3 * msg3.pose.pose.position.x)
        
        fused_y = (a1 * msg1.pose.pose.position.y) + \
                  (a2 * msg2.pose.pose.position.y) + \
                  (a3 * msg3.pose.pose.position.y)
        
        # For Z and orientation we use the priority filter
        # If a3 is active (No GPS), give priority to msg3, otherwise to msg1
        main_msg = msg3 if a3 > a1 else msg1
        
        fused_z = main_msg.pose.pose.position.z
        fused_quat = main_msg.pose.pose.orientation

        # Creation of the final Odometry message
        fused_odom = Odometry()
        fused_odom.header.stamp = self.get_clock().now().to_msg()
        fused_odom.header.frame_id = 'map'
        fused_odom.child_frame_id = 'base_link'

        fused_odom.pose.pose.position.x = fused_x
        fused_odom.pose.pose.position.y = fused_y
        fused_odom.pose.pose.position.z = fused_z
        fused_odom.pose.pose.orientation = fused_quat

        self.odom_pub.publish(fused_odom)

        # Broadcast of the transform (TF) for RViz
        t = TransformStamped()
        t.header.stamp = fused_odom.header.stamp
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'
        t.transform.translation.x = fused_x
        t.transform.translation.y = fused_y
        t.transform.translation.z = fused_z
        t.transform.rotation = fused_quat

        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = ComplementaryFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()