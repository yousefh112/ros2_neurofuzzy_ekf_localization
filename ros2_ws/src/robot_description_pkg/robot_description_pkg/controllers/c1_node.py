import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


def normalize_angle(angle: float) -> float:
    """Normalize angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


class C1Node(Node):
    def __init__(self):
        super().__init__('c1_node')

        self.rho = self.declare_parameter('rho', 1.0).value
        self.u_bar = self.declare_parameter('u_bar', 3.0).value
        self.output_topic = self.declare_parameter('output_topic', '/control/c1').value

        self.current_heading = None
        self.desired_heading = None

        self.heading_sub = self.create_subscription(
            Float64,
            '/robot/heading',
            self.heading_callback,
            10,
        )
        self.desired_heading_sub = self.create_subscription(
            Float64,
            '/robot/desired_heading',
            self.desired_heading_callback,
            10,
        )
        self.publisher = self.create_publisher(Float64, self.output_topic, 10)

        self.timer = self.create_timer(1.0, self.timer_callback)

        self.get_logger().info('c1_node ready: publishing to %s' % self.output_topic)

    def heading_callback(self, msg: Float64) -> None:
        self.current_heading = msg.data

    def desired_heading_callback(self, msg: Float64) -> None:
        self.desired_heading = msg.data

    def timer_callback(self) -> None:
        if self.current_heading is None or self.desired_heading is None:
            self.get_logger().warning('Waiting for /robot/heading and /robot/desired_heading messages')
            return

        self.rho = self.get_parameter('rho').value
        self.u_bar = self.get_parameter('u_bar').value

        beta = normalize_angle(self.current_heading  - self.desired_heading)
        c1_value = self.rho * math.copysign(1.0, beta) + self.u_bar

        output = Float64()
        output.data = c1_value
        self.publisher.publish(output)

        self.get_logger().info('c1(t)=%f, beta=%f, rho=%f, u_bar=%f' % (c1_value, beta, self.rho, self.u_bar))


def main(args=None):
    rclpy.init(args=args)
    node = C1Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
