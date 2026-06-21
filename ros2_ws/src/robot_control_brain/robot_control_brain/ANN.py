#!/usr/bin/env python3

import os
import math
import threading
import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point


class TrajectoryANN(nn.Module):
    """15→10→5→2 feedforward network (log-sigmoid hidden, linear output)."""
    def __init__(self, input_dim: int = 15, output_dim: int = 2):
        super().__init__()
        self.network = nn.Sequential(
            nn.Linear(input_dim, 10),
            nn.Sigmoid(),
            nn.Linear(10, 5),
            nn.Sigmoid(),
            nn.Linear(5, output_dim),
        )

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return self.network(x)


class OnlineTrainingNode(Node):

    WEIGHTS_PATH = os.path.expanduser('~/ros2_neurofuzzy_ekf_localization/ros2_ws/'
                                      'src/robot_control_brain/models/nn_weights.pt')

    def __init__(self):
        super().__init__('online_training_node')

        self.model     = TrajectoryANN()
        self.optimizer = optim.Adam(self.model.parameters(), lr=0.005)
        self.criterion = nn.MSELoss()

        self.in_mean : np.ndarray | None = None
        self.in_std  : np.ndarray | None = None
        self.tgt_mean: np.ndarray | None = None
        self.tgt_std : np.ndarray | None = None

        self.is_trained_at_least_once = False
        self.training_in_progress     = False
        self.lock = threading.Lock()

        self.MAX_BUFFER = 3000
        self.input_buffer : list[np.ndarray] = []
        self.target_buffer: list[np.ndarray] = []

        # Buffer fills only when GPS fix is active (outdoor phase).
        self.gps_fix_active = False

        # When GPS is lost, snapshot last GPS-corrected KF positions as anchor.
        # Indoor dead-reckoning shifts KF values out of the training distribution,
        # causing large ANN errors. The anchor keeps features 12-15 in-distribution.
        self._kf1_anchor: list[float] | None = None
        self._kf2_anchor: list[float] | None = None

        self._prev_stamp: float | None = None
        self._sigma_a_mag  = 0.0
        self._sigma_v_imu  = 0.0
        self._gamma_g      = 0.0
        self._sigma_omega  = 0.0
        self._sigma_vx_od  = 0.0
        self._sigma_vy_od  = 0.0

        # GPS deliberately not stored — ANN must run when GPS is unavailable.
        self._imu : list[float] | None = None
        self._odom: list[float] | None = None
        self._kf1 : list[float] | None = None
        self._kf2 : list[float] | None = None
        self._target: list[float] | None = None

        # Reject training samples when KF1 covariance overflows (P_xx > 5 m²).
        # Diverged EKF targets corrupt the ANN and produce large indoor errors.
        self._kf1_cov: float = 0.0

        self.create_subscription(Imu,       '/imu/data',         self._cb_imu,    10)
        self.create_subscription(Odometry,  '/odom',             self._cb_odom,   10)
        self.create_subscription(Odometry,  '/odometry/global',  self._cb_kf1,    10)
        self.create_subscription(Odometry,  '/odometry/global2', self._cb_kf2,    10)
        self.create_subscription(Odometry,  '/odometry/fused',   self._cb_target, 10)
        self.create_subscription(NavSatFix, '/gps/fix',          self._cb_gps_fix, 10)

        self.ann_pub    = self.create_publisher(Point, '/ann/trajectory',  10)
        self.target_pub = self.create_publisher(Point, '/ann/target_vis',  10)

        self.create_timer(0.1, self._control_loop)
        self.create_timer(5.0, self._trigger_training)

        self._load_weights()

        self.get_logger().info(
            '[ANN] Node started. Architecture: 15→10→5→2 | '
            f'Weights: {self.WEIGHTS_PATH}')

    def _cb_gps_fix(self, msg: NavSatFix) -> None:
        new_active = (msg.status.status >= NavSatStatus.STATUS_FIX)
        if self.gps_fix_active and not new_active:
            if self._kf1 is not None:
                self._kf1_anchor = list(self._kf1)
            if self._kf2 is not None:
                self._kf2_anchor = list(self._kf2)
            self.get_logger().info(
                f'[ANN] GPS lost — anchor saved: '
                f'KF1=({self._kf1_anchor[0]:.3f},{self._kf1_anchor[1]:.3f})  '
                f'KF2=({self._kf2_anchor[0]:.3f},{self._kf2_anchor[1]:.3f})'
                if self._kf1_anchor else '[ANN] GPS lost — no KF data yet to anchor')
        self.gps_fix_active = new_active

    def _cb_imu(self, msg: Imu) -> None:
        ax  = msg.linear_acceleration.x
        ay  = msg.linear_acceleration.y
        wz  = msg.angular_velocity.z
        now = rclpy.time.Time.from_msg(msg.header.stamp).nanoseconds * 1e-9

        if self._prev_stamp is None:
            self._prev_stamp = now
        dt = max(now - self._prev_stamp, 1e-4)
        self._prev_stamp = now

        a_mag = math.sqrt(ax * ax + ay * ay)
        v_imu = a_mag * dt

        self._sigma_a_mag  += a_mag
        self._sigma_v_imu  += v_imu
        self._gamma_g      += wz * dt
        self._sigma_omega  += wz

        self._imu = [ax, ay, wz]

    def _cb_odom(self, msg: Odometry) -> None:
        vx  = msg.twist.twist.linear.x
        vy  = msg.twist.twist.linear.y
        q   = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))

        self._sigma_vx_od += vx
        self._sigma_vy_od += vy
        self._odom = [vx, vy, yaw]

    def _cb_kf1(self, msg: Odometry) -> None:
        self._kf1 = [msg.pose.pose.position.x, msg.pose.pose.position.y]
        self._kf1_cov = msg.pose.covariance[0]   # P_xx [m²]

    def _cb_kf2(self, msg: Odometry) -> None:
        self._kf2 = [msg.pose.pose.position.x, msg.pose.pose.position.y]

    def _cb_target(self, msg: Odometry) -> None:
        self._target = [msg.pose.pose.position.x, msg.pose.pose.position.y]

    def _build_input(self) -> np.ndarray | None:
        if self._imu is None or self._odom is None or \
           self._kf1 is None or self._kf2 is None:
            return None

        ax, ay, wz  = self._imu
        vx, vy, yaw = self._odom

        # Use GPS-corrected KF anchor indoors to keep features in training distribution.
        if not self.gps_fix_active and self._kf1_anchor is not None:
            x1, y1 = self._kf1_anchor
            x2, y2 = self._kf2_anchor if self._kf2_anchor is not None \
                      else self._kf1_anchor
        else:
            x1, y1 = self._kf1
            x2, y2 = self._kf2

        v_dot_a = math.sqrt(ax * ax + ay * ay)
        x_dot_a = v_dot_a * (1.0 / 10.0)

        # 15 features: IMU×6, Odom×5, KF positions×4
        return np.array([
            v_dot_a,            #  1. acceleration magnitude
            self._sigma_a_mag,  #  2. cumulative accel mag
            x_dot_a,            #  3. IMU velocity magnitude (per step)
            self._sigma_v_imu,  #  4. cumulative IMU velocity
            self._gamma_g,      #  5. integrated gyro heading
            self._sigma_omega,  #  6. cumulative yaw rate
            vx,                 #  7. odom x velocity
            self._sigma_vx_od,  #  8. cumulative odom x velocity
            vy,                 #  9. odom y velocity
            self._sigma_vy_od,  # 10. cumulative odom y velocity
            yaw,                # 11. heading from odom
            x1,                 # 12. x_kf1
            y1,                 # 13. y_kf1
            x2,                 # 14. x_kf2
            y2,                 # 15. y_kf2
        ], dtype=np.float32)

    def _control_loop(self) -> None:
        raw_input = self._build_input()
        if raw_input is None:
            missing = []
            if self._imu  is None: missing.append('imu')
            if self._odom is None: missing.append('odom')
            if self._kf1  is None: missing.append('kf1')
            if self._kf2  is None: missing.append('kf2')
            self.get_logger().warn(
                f'[ANN] Waiting for: {missing}', throttle_duration_sec=3.0)
            return

        _EKF_COV_MAX_M2 = 5.0
        if self.gps_fix_active and self._target is not None:
            if self._kf1_cov > _EKF_COV_MAX_M2:
                self.get_logger().warn(
                    f'[ANN] Sample rejected: KF1 P_xx={self._kf1_cov:.2f} m²',
                    throttle_duration_sec=5.0)
            else:
                raw_target = np.array(self._target, dtype=np.float32)
                self.input_buffer.append(raw_input.copy())
                self.target_buffer.append(raw_target)
                if len(self.input_buffer) > self.MAX_BUFFER:
                    self.input_buffer.pop(0)
                    self.target_buffer.pop(0)
                self.target_pub.publish(
                    Point(x=float(raw_target[0]), y=float(raw_target[1]), z=0.0))

        if not self.is_trained_at_least_once or self.in_mean is None:
            return

        norm_input = (raw_input - self.in_mean) / self.in_std
        tensor_in  = torch.tensor(norm_input, dtype=torch.float32).unsqueeze(0)

        with self.lock:
            with torch.no_grad():
                norm_out = self.model(tensor_in).numpy().flatten()

        raw_out = (norm_out * self.tgt_std) + self.tgt_mean
        self.ann_pub.publish(
            Point(x=float(raw_out[0]), y=float(raw_out[1]), z=0.0))

    def _trigger_training(self) -> None:
        if self.training_in_progress or len(self.input_buffer) < 500:
            return

        inputs_copy  = np.array(self.input_buffer,  dtype=np.float32)
        targets_copy = np.array(self.target_buffer, dtype=np.float32)

        self.training_in_progress = True
        t = threading.Thread(
            target=self._training_worker,
            args=(inputs_copy, targets_copy),
            daemon=True)
        t.start()

    def _training_worker(self, inputs: np.ndarray, targets: np.ndarray) -> None:
        try:
            in_mean  = inputs.mean(axis=0)
            in_std   = inputs.std(axis=0)  + 1e-6
            tgt_mean = targets.mean(axis=0)
            tgt_std  = targets.std(axis=0) + 1e-6   # guard against NaN when stationary

            norm_inputs  = (inputs  - in_mean)  / in_std
            norm_targets = (targets - tgt_mean) / tgt_std

            X = torch.tensor(norm_inputs,  dtype=torch.float32)
            Y = torch.tensor(norm_targets, dtype=torch.float32)

            self.model.train()
            for epoch in range(20):
                self.optimizer.zero_grad()
                preds = self.model(X)
                loss  = self.criterion(preds, Y)
                loss.backward()
                self.optimizer.step()

            with self.lock:
                self.model.eval()
                self.in_mean  = in_mean
                self.in_std   = in_std
                self.tgt_mean = tgt_mean
                self.tgt_std  = tgt_std
                self.is_trained_at_least_once = True

            self.get_logger().info(
                f'[ANN] Training complete. Loss={loss.item():.4f}  N={len(X)}')

        except Exception as exc:
            self.get_logger().error(f'[ANN] Training error: {exc}')
        finally:
            self.training_in_progress = False

    def _load_weights(self) -> None:
        if not os.path.exists(self.WEIGHTS_PATH):
            self.get_logger().info('[ANN] No persisted weights — starting fresh.')
            return
        try:
            # weights_only=False required: PyTorch 2.6 cannot allowlist numpy dtype
            # aliases even for fully trusted checkpoints written by this node.
            checkpoint = torch.load(self.WEIGHTS_PATH, weights_only=False)
            self.model.load_state_dict(checkpoint['model_state'])
            if 'in_mean' in checkpoint:
                self.in_mean  = np.array(checkpoint['in_mean'],  dtype=np.float32)
                self.in_std   = np.array(checkpoint['in_std'],   dtype=np.float32)
                self.tgt_mean = np.array(checkpoint['tgt_mean'], dtype=np.float32)
                self.tgt_std  = np.array(checkpoint['tgt_std'],  dtype=np.float32)
                self.is_trained_at_least_once = True
            self.model.eval()
            self.get_logger().info(f'[ANN] Loaded weights from {self.WEIGHTS_PATH}')
        except Exception as exc:
            self.get_logger().warn(f'[ANN] Failed to load weights: {exc}')

    def save_weights(self) -> None:
        if not self.is_trained_at_least_once:
            self.get_logger().info('[ANN] No trained weights to save.')
            return
        try:
            os.makedirs(os.path.dirname(self.WEIGHTS_PATH), exist_ok=True)
            # Store normalization params as plain lists so checkpoints load
            # cleanly with weights_only=True on future PyTorch versions.
            checkpoint = {
                'model_state': self.model.state_dict(),
                'in_mean' : self.in_mean.tolist()  if self.in_mean  is not None else None,
                'in_std'  : self.in_std.tolist()   if self.in_std   is not None else None,
                'tgt_mean': self.tgt_mean.tolist() if self.tgt_mean is not None else None,
                'tgt_std' : self.tgt_std.tolist()  if self.tgt_std  is not None else None,
            }
            torch.save(checkpoint, self.WEIGHTS_PATH)
            self.get_logger().info(f'[ANN] Weights saved → {self.WEIGHTS_PATH}')
        except Exception as exc:
            self.get_logger().error(f'[ANN] Failed to save weights: {exc}')


def main(args=None):
    rclpy.init(args=args)
    node = OnlineTrainingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save_weights()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
