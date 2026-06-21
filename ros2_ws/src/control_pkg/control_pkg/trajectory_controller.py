import math
from dataclasses import dataclass
from typing import Dict, Tuple

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf_transformations import euler_from_quaternion


@dataclass(frozen=True)
class DoorGate:
    """Geometric descriptor for one door opening (map frame).

    wall_axis     : 'x' = N-S wall (robot crosses E-W)
                    'y' = E-W wall (robot crosses N-S)
    wall_coord    : Wall plane coordinate [m].
    approach_side : 'west'|'east' for x-walls; 'south'|'north' for y-walls.
    align_center  : Door centre on orthogonal axis [m].
    safe_half_width: Conservative half-width of door gap [m].
    """
    name:            str
    wall_axis:       str
    wall_coord:      float
    approach_side:   str
    align_center:    float
    safe_half_width: float


DOOR_GATE_WPS: Dict[int, DoorGate] = {
    # Pass 1 (WP 11–50)
    11:  DoorGate('s_entry_p1_app', 'y', -4.149, 'south',  0.161, 0.45),
    12:  DoorGate('s_entry_p1_in',  'y', -4.149, 'south',  0.161, 0.45),
    50:  DoorGate('s_exit_p1',      'y', -4.149, 'north',  0.161, 0.45),
    16:  DoorGate('door_B_WE_p1',   'x',  3.849, 'west',  -3.43, 0.40),
    29:  DoorGate('door_B_EW_p1',   'x',  3.849, 'east',  -3.43, 0.40),
    21:  DoorGate('door_B2_SN_p1',  'y',  0.201, 'south',  4.705, 0.40),
    25:  DoorGate('door_B2_NS_p1',  'y',  0.201, 'north',  4.705, 0.40),
    33:  DoorGate('door_A_EW_p1',   'x', -3.501, 'east',  -3.47, 0.40),
    46:  DoorGate('door_A_WE_p1',   'x', -3.501, 'west',  -3.47, 0.40),
    38:  DoorGate('door_A2_SN_p1',  'y',  0.211, 'south', -4.741, 0.40),
    42:  DoorGate('door_A2_NS_p1',  'y',  0.211, 'north', -4.741, 0.40),
    # Pass 2 (WP 60–94)
    60:  DoorGate('s_entry_p2_app', 'y', -4.149, 'south',  0.161, 0.45),
    61:  DoorGate('s_entry_p2_in',  'y', -4.149, 'south',  0.161, 0.45),
    94:  DoorGate('s_exit_p2',      'y', -4.149, 'north',  0.161, 0.45),
    65:  DoorGate('door_A_EW_p2',   'x', -3.501, 'east',  -3.47, 0.40),
    90:  DoorGate('door_A_WE_p2',   'x', -3.501, 'west',  -3.47, 0.40),
    71:  DoorGate('door_A2_SN_p2',  'y',  0.211, 'south', -4.741, 0.40),
    86:  DoorGate('door_A2_NS_p2',  'y',  0.211, 'north', -4.741, 0.40),
    75:  DoorGate('door_C2_WE_p2',  'x', -3.510, 'west',   0.902, 0.40),
    81:  DoorGate('door_C2_EW_p2',  'x', -3.510, 'east',   0.902, 0.40),
    # Pass 3 (WP 108–129)
    108: DoorGate('s_entry_p3_app', 'y', -4.149, 'south',  0.161, 0.45),
    109: DoorGate('s_entry_p3_in',  'y', -4.149, 'south',  0.161, 0.45),
    129: DoorGate('s_exit_p3',      'y', -4.149, 'north',  0.161, 0.45),
    113: DoorGate('door_B_WE_p3',   'x',  3.849, 'west',  -3.43, 0.40),
    125: DoorGate('door_B_EW_p3',   'x',  3.849, 'east',  -3.43, 0.40),
    117: DoorGate('door_B2_SN_p3',  'y',  0.201, 'south',  4.705, 0.40),
    121: DoorGate('door_B2_NS_p3',  'y',  0.201, 'north',  4.705, 0.40),
}

# WP index ranges that use /odom directly as position source.
# Map frame == odom frame (same spawn origin). GPS bias (0.3–0.9 m, unstable)
# must NOT be added — it degrades accuracy and causes early door-guard fires.
INDOOR_RANGES: Tuple[Tuple[int, int], ...] = ((11, 50), (60, 94), (108, 129))

# Jump filter for /odometry/bt_fused (outdoor only; /odom needs no filter).
JUMP_REJECT_M = 1.00
JUMP_WARN_M   = 0.50
# Force-accept after this many consecutive rejects: ANN transient spikes
# resolve in <1 s (~10 rejects), but permanent GPS drift never resolves.
# 50 rejects = 5 s at 10 Hz — catches drift without discarding valid spikes.
JUMP_REJECT_FALLBACK_COUNT = 50

# Minimum forward speed fraction: v = v_max * max(cos(theta_err), V_MIN_FACTOR)
V_MIN_FACTOR = 0.15

RECOVERY_PHASE1_SECS = 1.5
RECOVERY_PHASE2_SECS = 1.5
RECOVERY_TOTAL_SECS  = RECOVERY_PHASE1_SECS + RECOVERY_PHASE2_SECS
RECOVERY_VEL         = -0.20

PROGRESS_DELTA_M      = 0.08
PROGRESS_TIMEOUT_SECS = 8.0


class TrajectoryController(Node):

    def __init__(self):
        super().__init__('trajectory_controller')

        self.declare_parameter('waypoints',
            [4.0,1.0, 5.0,5.0, 0.0,5.0, -5.0,5.0, -5.0,0.0,
             -5.0,-5.0, 0.0,-5.0, 5.0,-5.0, 5.0,0.0])
        self.declare_parameter('linear_speed',        0.5)
        self.declare_parameter('angular_gain',        0.50)
        self.declare_parameter('max_angular_speed',   0.50)
        self.declare_parameter('distance_threshold',  0.45)
        self.declare_parameter('angle_threshold',     0.70)
        self.declare_parameter('loop_trajectory',     False)
        self.declare_parameter('align_guard_enabled', True)
        self.declare_parameter('align_guard_threshold', 0.20)

        flat = self.get_parameter('waypoints').value
        self.waypoints  = [(flat[i], flat[i+1]) for i in range(0, len(flat), 2)]
        self._v         = self.get_parameter('linear_speed').value
        self._K_w       = self.get_parameter('angular_gain').value
        self._w_max     = self.get_parameter('max_angular_speed').value
        self._d_th      = self.get_parameter('distance_threshold').value
        self._theta_th  = self.get_parameter('angle_threshold').value
        self._loop      = self.get_parameter('loop_trajectory').value
        self._guard_en  = self.get_parameter('align_guard_enabled').value
        self._eps_th    = self.get_parameter('align_guard_threshold').value

        self._bt_x    = 0.0
        self._bt_y    = 0.0
        self._bt_yaw  = 0.0
        self._has_bt  = False
        self._jump_n  = 0
        # Arm after indoor→outdoor WP transition: the jump-filter last_pos is
        # stale (frozen since indoor entry). Force-accept the first bt_fused
        # callback to reset it to the current GPS re-acquisition value.
        self._pending_bt_reset: bool = False

        self._odom_x   = 0.0
        self._odom_y   = 0.0
        self._odom_yaw = 0.0
        self._has_odom = False

        self.wp_idx     = 0
        self._done      = False
        self._last_log  = self.get_clock().now()
        self._guard_name = ''

        self._in_recovery      = False
        self._recovery_elapsed = 0.0

        self._prog_min_dist = math.inf
        self._prog_last_t   = None

        self._pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odometry/bt_fused', self._bt_cb, 10)
        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        self.create_timer(0.1, self._loop_cb)

        self.get_logger().info(
            f'TrajectoryController ready — {len(self.waypoints)} WPs | '
            f'guard v4 | eps={self._eps_th}m')

    @property
    def _indoor(self) -> bool:
        return any(lo <= self.wp_idx <= hi for lo, hi in INDOOR_RANGES)

    @property
    def px(self) -> float:
        if self._indoor and self._has_odom:
            return self._odom_x
        return self._bt_x

    @property
    def py(self) -> float:
        if self._indoor and self._has_odom:
            return self._odom_y
        return self._bt_y

    @property
    def pyaw(self) -> float:
        if self._indoor and self._has_odom:
            return self._odom_yaw
        return self._bt_yaw

    def _bt_cb(self, msg: Odometry) -> None:
        nx  = msg.pose.pose.position.x
        ny  = msg.pose.pose.position.y
        q   = msg.pose.pose.orientation
        _, _, nyaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

        if not self._has_bt:
            self._bt_x, self._bt_y, self._bt_yaw = nx, ny, nyaw
            self._has_bt = True
            self.get_logger().info('/odometry/bt_fused received. Controller armed.')
            return

        jump = math.hypot(nx - self._bt_x, ny - self._bt_y)

        if self._pending_bt_reset:
            self._pending_bt_reset = False
            old_x, old_y = self._bt_x, self._bt_y
            self._jump_n = 0
            self._bt_x, self._bt_y, self._bt_yaw = nx, ny, nyaw
            self.get_logger().info(
                f'[JUMP_RESET] Indoor→outdoor: '
                f'({old_x:.2f},{old_y:.2f})→({nx:.2f},{ny:.2f}) Δ={jump:.2f}m')
            return

        if self._jump_n >= JUMP_REJECT_FALLBACK_COUNT:
            n = self._jump_n
            old_x, old_y = self._bt_x, self._bt_y
            self._jump_n = 0
            self._bt_x, self._bt_y, self._bt_yaw = nx, ny, nyaw
            self.get_logger().warn(
                f'[JUMP_RESET] {n} consecutive rejects — GPS drift accepted: '
                f'({old_x:.2f},{old_y:.2f})→({nx:.2f},{ny:.2f}) Δ={jump:.2f}m')
            return

        if jump > JUMP_REJECT_M:
            self._jump_n += 1
            self.get_logger().warn(
                f'[JUMP_REJECT] delta={jump:.2f}m '
                f'({self._bt_x:.2f},{self._bt_y:.2f})->'
                f'({nx:.2f},{ny:.2f}) [#{self._jump_n}]')
            return
        if jump > JUMP_WARN_M:
            self.get_logger().info(
                f'[JUMP_WARN] delta={jump:.2f}m accepted')

        self._jump_n = 0
        self._bt_x, self._bt_y, self._bt_yaw = nx, ny, nyaw

    def _odom_cb(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self._odom_x   = msg.pose.pose.position.x
        self._odom_y   = msg.pose.pose.position.y
        self._odom_yaw = yaw
        self._has_odom = True

    def _guard(self, gate: DoorGate) -> Tuple[bool, str]:
        if gate.wall_axis == 'x':
            robot_wall  = self.px
            robot_align = self.py
            crossed = (robot_wall > gate.wall_coord
                       if gate.approach_side == 'west'
                       else robot_wall < gate.wall_coord)
        else:
            robot_wall  = self.py
            robot_align = self.px
            crossed = (robot_wall > gate.wall_coord
                       if gate.approach_side == 'south'
                       else robot_wall < gate.wall_coord)

        align_err = abs(robot_align - gate.align_center)

        if crossed and align_err <= self._eps_th:
            return False, f'crossed+aligned eps={align_err:.3f}m'
        if crossed:
            return True, f'crossed-misaligned eps={align_err:.3f}m'
        return True, f'not-crossed eps={align_err:.3f}m'

    def _reset_progress(self) -> None:
        self._prog_min_dist = math.inf
        self._prog_last_t   = None

    def _check_progress(self, dist: float) -> bool:
        now = self.get_clock().now()
        if self._prog_last_t is None:
            self._prog_last_t   = now
            self._prog_min_dist = dist
            return False

        if dist < self._prog_min_dist - PROGRESS_DELTA_M:
            self._prog_min_dist = dist
            self._prog_last_t   = now
            return False

        stall = (now - self._prog_last_t).nanoseconds / 1e9
        if stall > PROGRESS_TIMEOUT_SECS:
            src = 'odom' if (self._indoor and self._has_odom) else 'bt'
            self.get_logger().warn(
                f'[STUCK] WP{self.wp_idx}: stalled {stall:.1f}s at '
                f'({self.px:.2f},{self.py:.2f}) [{src}]  '
                f'dist={dist:.2f}m. Triggering recovery.')
            return True
        return False

    def _run_recovery(self) -> None:
        self._recovery_elapsed += 0.1
        twist = Twist()

        if self._recovery_elapsed <= RECOVERY_PHASE1_SECS:
            twist.linear.x  = RECOVERY_VEL
            twist.angular.z = 0.0
        else:
            tx, ty = self.waypoints[self.wp_idx]
            t_yaw  = math.atan2(ty - self.py, tx - self.px)
            a_err  = math.atan2(math.sin(t_yaw - self.pyaw),
                                math.cos(t_yaw - self.pyaw))
            twist.angular.z = max(-self._w_max,
                                  min(self._w_max, 1.5 * self._K_w * a_err))

        self._pub.publish(twist)

        if self._recovery_elapsed >= RECOVERY_TOTAL_SECS:
            self._in_recovery      = False
            self._recovery_elapsed = 0.0
            self._guard_name       = ''
            self._reset_progress()
            self.get_logger().info(f'[RECOVERY] Done. Resuming WP{self.wp_idx}.')

    def _loop_cb(self) -> None:
        if not self._has_bt or self._done:
            return

        if self._in_recovery:
            self._run_recovery()
            return

        if self.wp_idx >= len(self.waypoints):
            if self._loop:
                self.wp_idx = 0
            else:
                self._done = True
                self.get_logger().info('Trajectory completed!')
                self._pub.publish(Twist())
            return

        tx, ty = self.waypoints[self.wp_idx]

        guard_active = False
        guard_name   = ''

        if self._guard_en and self.wp_idx in DOOR_GATE_WPS:
            gate = DOOR_GATE_WPS[self.wp_idx]
            guard_active, reason = self._guard(gate)
            if guard_active:
                guard_name = gate.name
                if self._guard_name != guard_name:
                    self.get_logger().warn(
                        f'[GUARD] {guard_name} WP{self.wp_idx} {reason}')
                    self._guard_name = guard_name
            else:
                if self._guard_name:
                    self.get_logger().info(
                        f'[GUARD] Cleared {self._guard_name} ({reason})')
                    self._guard_name = ''

        dx   = tx - self.px
        dy   = ty - self.py
        dist = math.hypot(dx, dy)

        t_yaw = math.atan2(dy, dx)
        a_err = math.atan2(math.sin(t_yaw - self.pyaw),
                           math.cos(t_yaw - self.pyaw))

        if self._check_progress(dist):
            self._in_recovery      = True
            self._recovery_elapsed = 0.0
            self._reset_progress()
            self._pub.publish(Twist())
            return

        now = self.get_clock().now()
        if (now - self._last_log).nanoseconds > 1e9:
            src  = 'odom' if (self._indoor and self._has_odom) else 'bt'
            gtag = f' [GUARD:{guard_name}]' if guard_active else ''
            self.get_logger().info(
                f'[{src}] ({self.px:.2f},{self.py:.2f})  '
                f'WP{self.wp_idx}:({tx:.2f},{ty:.2f})  '
                f'dist:{dist:.2f}m  th:{math.degrees(a_err):.1f}deg{gtag}')
            self._last_log = now

        if dist < self._d_th and not guard_active:
            self.get_logger().info(f'WP {self.wp_idx} captured: ({tx:.2f},{ty:.2f})')
            was_indoor = self._indoor
            self.wp_idx += 1
            if was_indoor and not self._indoor:
                # Arm force-accept: jump-filter last_pos is stale (frozen since
                # indoor entry). Next bt_fused callback resets it to current GPS.
                self._pending_bt_reset = True
                self.get_logger().info(
                    f'[JUMP_RESET_ARM] WP{self.wp_idx-1}→WP{self.wp_idx}: '
                    f'indoor→outdoor. bt_fused filter will reset on next callback.')
            self._reset_progress()
            return

        # Always-forward pursuit: v = v_max * max(cos(θ_err), V_MIN_FACTOR)
        twist = Twist()
        twist.angular.z = max(-self._w_max,
                              min(self._w_max, self._K_w * a_err))
        twist.linear.x  = self._v * max(math.cos(a_err), V_MIN_FACTOR)
        self._pub.publish(twist)

    def destroy_node(self):
        self._pub.publish(Twist())
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
