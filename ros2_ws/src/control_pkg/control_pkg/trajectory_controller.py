"""
Trajectory controller node for ROS2.

Subscribes to odometry and publishes velocity commands to navigate through
a predefined set of waypoints. Implements a control loop that prioritizes
rotation when the angular error is large, then moves forward with reduced
angular velocity during linear motion.
"""

import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf_transformations import euler_from_quaternion


class TrajectoryController(Node):
    """
    ROS2 node for trajectory control using waypoint navigation.

    Subscribes to odometry, publishes velocity commands to navigate through
    waypoints while stress-testing the localization system with complex paths.
    """

    def __init__(self):
        """Initialize the trajectory controller node."""
        super().__init__('trajectory_controller')

        # Declare parameters
        self.declare_parameter(
            'waypoints',
            [
                4.0, 1.0,
                5.0, 5.0,
                0.0, 5.0,
                -5.0, 5.0,
                -5.0, 0.0,
                -5.0, -5.0,
                0.0, -5.0,
                5.0, -5.0,
                5.0, 0.0,
            ]
        )
        self.declare_parameter('linear_speed', 0.4)
        self.declare_parameter('angular_gain', 1.0)
        self.declare_parameter('max_angular_speed', 1.0)
        self.declare_parameter('distance_threshold', 0.15)
        self.declare_parameter('angle_threshold', 0.15)
        self.declare_parameter('loop_trajectory', False)

        # Get parameters
        waypoints_flat = self.get_parameter('waypoints').value
        self.waypoints = [
            (waypoints_flat[i], waypoints_flat[i + 1])
            for i in range(0, len(waypoints_flat), 2)
        ]
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_gain = self.get_parameter('angular_gain').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.distance_threshold = self.get_parameter('distance_threshold').value
        self.angle_threshold = self.get_parameter('angle_threshold').value
        self.loop_trajectory = self.get_parameter('loop_trajectory').value

        # State variables
        self.current_waypoint_idx = 0
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.trajectory_completed = False
        self.last_log_time = self.get_clock().now()

        # Publishers and subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )

        # Timer for control loop at 10 Hz
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info(
            f'Trajectory controller initialized with {len(self.waypoints)} waypoints'
        )
        self.get_logger().info(
            f'Linear speed: {self.linear_speed} m/s, '
            f'Angular gain: {self.angular_gain}, '
            f'Max angular speed: {self.max_angular_speed} rad/s, '
            f'Distance threshold: {self.distance_threshold} m, '
            f'Angle threshold: {self.angle_threshold} rad, '
            f'Loop trajectory: {self.loop_trajectory}'
        )

    def odom_callback(self, msg: Odometry) -> None:
        """
        Callback for odometry subscription.

        Extracts position and orientation from odometry message.

        Args:
            msg: Odometry message containing pose and twist information.
        """
        # Extract position
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        # Extract yaw from quaternion
        quat = msg.pose.pose.orientation
        _, _, self.current_yaw = euler_from_quaternion(
            [quat.x, quat.y, quat.z, quat.w]
        )

    def control_loop(self) -> None:
        """
        Main control loop executed at 10 Hz.

        Implements waypoint navigation with rotation priority and
        logging at 1 Hz to avoid terminal flooding.
        """
        if self.trajectory_completed:
            return

        # Check if all waypoints completed
        if self.current_waypoint_idx >= len(self.waypoints):
            if self.loop_trajectory:
                self.current_waypoint_idx = 0
                self.trajectory_completed = False
                self.get_logger().info('Restarting trajectory')
                return
            self.trajectory_completed = True
            self.get_logger().info('Trajectory completed!')
            twist_msg = Twist()
            self.cmd_vel_pub.publish(twist_msg)
            return

        # Get current waypoint
        target_x, target_y = self.waypoints[self.current_waypoint_idx]

        # Compute distance and angle errors
        dx = target_x - self.current_x
        dy = target_y - self.current_y
        distance = math.sqrt(dx**2 + dy**2)

        # Target yaw
        target_yaw = math.atan2(dy, dx)

        # Compute angle error (shortest path on unit circle)
        angle_error = math.atan2(
            math.sin(target_yaw - self.current_yaw),
            math.cos(target_yaw - self.current_yaw)
        )

        # Log at 1 Hz (every 10 iterations at 10 Hz)
        current_time = self.get_clock().now()
        if (current_time - self.last_log_time).nanoseconds > 1e9:
            self.get_logger().info(
                f'Position: ({self.current_x:.2f}, {self.current_y:.2f}), '
                f'Waypoint {self.current_waypoint_idx}: '
                f'({target_x:.2f}, {target_y:.2f}), '
                f'Distance: {distance:.2f} m'
            )
            self.last_log_time = current_time

        # Check if waypoint reached
        if distance < self.distance_threshold:
            self.get_logger().info(
                f'Waypoint {self.current_waypoint_idx} reached: '
                f'({target_x:.2f}, {target_y:.2f})'
            )
            self.current_waypoint_idx += 1
            return

        # Control logic
        twist_msg = Twist()

        # Rotation priority: if angle error is large, only rotate
        if abs(angle_error) > self.angle_threshold:
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = self.angular_gain * angle_error
        else:
            # Move forward with reduced angular velocity
            twist_msg.linear.x = self.linear_speed
            twist_msg.angular.z = 0.5 * self.angular_gain * angle_error

        twist_msg.angular.z = max(
            -self.max_angular_speed,
            min(self.max_angular_speed, twist_msg.angular.z)
        )
        self.cmd_vel_pub.publish(twist_msg)

    def destroy_node(self):
        """Cleanup node resources."""
        super().destroy_node()


def main(args=None):
    """
    Main entry point for the trajectory controller node.

    Initializes ROS2, creates the trajectory controller node, spins it,
    and ensures proper cleanup.

    Args:
        args: Command line arguments (default: None).
    """
    rclpy.init(args=args)
    trajectory_controller = TrajectoryController()

    try:
        rclpy.spin(trajectory_controller)
    except KeyboardInterrupt:
        pass
    finally:
        trajectory_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
