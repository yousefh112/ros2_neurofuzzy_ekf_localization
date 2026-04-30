import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class C2Node(Node):
    def __init__(self):
        super().__init__('c2_node')

        self.f = self.declare_parameter('F', 0.016).value
        self.output_topic = self.declare_parameter('output_topic', '/control/c2').value

        self.t = 0
        self.publisher = self.create_publisher(Float64, self.output_topic, 10)
        self.timer = self.create_timer(1.0, self.timer_callback)

        self.get_logger().info('c2_node ready: publishing to %s' % self.output_topic)

    def timer_callback(self) -> None:
        self.f = self.get_parameter('F').value

        c2_value = 10.0 * math.sin(2.0 * math.pi * self.f * self.t)
        output = Float64()
        output.data = c2_value
        self.publisher.publish(output)

        self.get_logger().info('t=%d c2(t)=%f F=%f' % (self.t, c2_value, self.f))
        self.t += 1


def main(args=None):
    rclpy.init(args=args)
    node = C2Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
