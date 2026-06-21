#!/usr/bin/env python3
"""
Offline validation from rosbag + launch log.

USAGE:
  python3 validate_run.py                          # auto-detect latest bag
  python3 validate_run.py --bag /path/to/bag_dir
  python3 validate_run.py --bag /path/to/bag --log /path/to/launch_log.txt

INSTALL (one-time):  pip install rosbags --break-system-packages

Default bag dir: ~/ros2_neurofuzzy_ekf_localization/ros2_ws/src/bt_orchestrator_pkg/
Default log:     ~/launch_log.txt
"""

import argparse
import glob
import math
import os
import re
import sys

import numpy as np
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore

DEFAULT_BAGS_DIR = os.path.expanduser(
    '~/ros2_neurofuzzy_ekf_localization/ros2_ws/src/bt_orchestrator_pkg'
)
DEFAULT_LOG = os.path.expanduser('~/launch_log.txt')

KPI = {
    'spin_events_max':        0,      # ω_z > 0.5 rad/s AND v_x < 0.05 m/s
    'gps_offset_max_m':       0.50,   # max(|odom − gps|) < 0.50 m
    'waypoints_required':     92,     # 92/92 WPs reached (from launch log)
    'indoor_transitions_min': 2,      # ≥ 2 OUT→IN or IN→OUT state changes
    'ann_nan_allowed':        0,      # /odometry/bt_fused NaN count = 0
    'ekf_cov_diverge_max':    5.0,    # σ²_x or σ²_y in /odometry/global > 5 m²
    'run_time_max_s':         960.0,  # ≤ 16 min
    'ann_samples_min':        500,    # ANN buffer must reach ≥ 500 (training fires)
    'ann_loss_max':           0.10,   # last training MSE < 0.10
    'rmse_bt_vs_gt_max_m':    0.80,   # RMSE /odometry/bt_fused vs /gazebo/model_states
}

TYPESTORE = get_typestore(Stores.ROS2_HUMBLE)


def find_latest_bag(bags_dir: str) -> str:
    """Return the most recently modified bag directory under bags_dir."""
    candidates = sorted(
        [d for d in glob.glob(os.path.join(bags_dir, 'run_bag*'))
         if os.path.isdir(d)],
        key=os.path.getmtime,
        reverse=True,
    )
    if not candidates:
        # Also check if bags_dir itself is a bag
        if os.path.exists(os.path.join(bags_dir, 'metadata.yaml')):
            return bags_dir
        raise FileNotFoundError(
            f'No run_bag* directories found in {bags_dir}\n'
            f'Record a bag first:\n'
            f'  ros2 bag record /odometry/global /odometry/global2 '
            f'/odometry/local /odometry/local2 /odometry/gps '
            f'/odometry/bt_fused /odom /cmd_vel /gps/fix /imu/data '
            f'/bt/indoor_detection /ann/diagnostics /gazebo/model_states '
            f'/tf /tf_static -o run_bag_$(date +%Y%m%d_%H%M%S)'
        )
    return candidates[0]


def get_connections(reader, topic: str):
    return [c for c in reader.connections if c.topic == topic]


def iter_topic(reader, topic: str):
    conns = get_connections(reader, topic)
    if not conns:
        return
    for conn, ts, raw in reader.messages(connections=conns):
        msg = TYPESTORE.deserialize_cdr(raw, conn.msgtype)
        yield ts, msg


class RunValidator:

    def __init__(self, bag_path: str, log_path: str):
        self.bag_path = bag_path
        self.log_path = log_path
        self.results  = {}   # name → (value, limit, op, status_str)
        self._all_pass = True

    def _record(self, name: str, value, limit, op: str):
        if isinstance(value, float) and (math.isnan(value) or math.isinf(value)):
            status = '⚠  NO DATA'
        elif op == '≤':
            status = '✅ PASS' if value <= limit else '❌ FAIL'
            if value > limit:
                self._all_pass = False
        else:  # ≥
            status = '✅ PASS' if value >= limit else '❌ FAIL'
            if value < limit:
                self._all_pass = False
        self.results[name] = (value, limit, op, status)

    def _check_spin_events(self, reader) -> int:
        spin = 0
        for ts, msg in iter_topic(reader, '/cmd_vel'):
            if (abs(msg.angular.z) > 0.50 and abs(msg.linear.x) < 0.05):
                spin += 1
        self._record('Spin events', spin, KPI['spin_events_max'], '≤')
        return spin

    def _check_gps_offset(self, reader) -> dict:
        odom_buf, gps_buf = [], []
        for ts, msg in iter_topic(reader, '/odom'):
            odom_buf.append((ts, msg.pose.pose.position.x, msg.pose.pose.position.y))
        for ts, msg in iter_topic(reader, '/odometry/gps'):
            gps_buf.append((ts, msg.pose.pose.position.x, msg.pose.pose.position.y))

        if not odom_buf or not gps_buf:
            self._record('GPS offset max [m]', float('nan'), KPI['gps_offset_max_m'], '≤')
            return {}

        deltas = []
        for g_ts, gx, gy in gps_buf:
            closest = min(odom_buf, key=lambda o: abs(o[0] - g_ts))
            deltas.append(math.hypot(closest[1] - gx, closest[2] - gy))

        stats = {'mean': np.mean(deltas), 'max': np.max(deltas), 'std': np.std(deltas)}
        self._record('GPS offset max [m]', float(stats['max']), KPI['gps_offset_max_m'], '≤')
        return stats

    def _check_ann_validity(self, reader):
        nan_count, total = 0, 0
        for ts, msg in iter_topic(reader, '/odometry/bt_fused'):
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            total += 1
            if (math.isnan(x) or math.isnan(y) or
                    math.isinf(x) or math.isinf(y)):
                nan_count += 1
        self._record('ANN NaN msgs', nan_count, KPI['ann_nan_allowed'], '≤')
        return {'nan_count': nan_count, 'total': total}

    def _check_ekf_covariance(self, reader) -> float:
        max_cov = 0.0
        for ts, msg in iter_topic(reader, '/odometry/global'):
            cov_xx = msg.pose.covariance[0]
            cov_yy = msg.pose.covariance[7]
            max_cov = max(max_cov, cov_xx, cov_yy)
        self._record('EKF cov max [m²]', max_cov, KPI['ekf_cov_diverge_max'], '≤')
        return max_cov

    def _check_run_time(self, reader) -> float:
        ts_list = [ts for conn, ts, raw in reader.messages()]
        if not ts_list:
            self._record('Run time [s]', float('nan'), KPI['run_time_max_s'], '≤')
            return float('nan')
        duration = (max(ts_list) - min(ts_list)) / 1e9
        self._record('Run time [s]', duration, KPI['run_time_max_s'], '≤')
        return duration

    def _check_transitions(self, reader) -> int:
        transitions, last = 0, None
        for ts, msg in iter_topic(reader, '/bt/indoor_detection'):
            state = msg.data
            if last is not None and state != last:
                transitions += 1
            last = state
        self._record('Indoor transitions', transitions, KPI['indoor_transitions_min'], '≥')
        return transitions

    def _check_rmse_vs_gt(self, reader) -> float:
        bt_buf, gt_buf = [], []
        for ts, msg in iter_topic(reader, '/odometry/bt_fused'):
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            if not (math.isnan(x) or math.isnan(y)):
                bt_buf.append((ts, x, y))

        for ts, msg in iter_topic(reader, '/gazebo/model_states'):
            try:
                idx = msg.name.index('turtlebot3_burger')
            except ValueError:
                continue
            p = msg.pose[idx].position
            gt_buf.append((ts, p.x, p.y))

        if not bt_buf or not gt_buf:
            self._record('RMSE vs GT [m]', float('nan'), KPI['rmse_bt_vs_gt_max_m'], '≤')
            print('  ⚠  /gazebo/model_states not in bag — world plugin not yet active')
            print('     Run with the updated indoor_outdoor.world to enable GT.')
            return float('nan')

        errors_sq = []
        for b_ts, bx, by in bt_buf:
            g = min(gt_buf, key=lambda g: abs(g[0] - b_ts))
            errors_sq.append((bx - g[1])**2 + (by - g[2])**2)
        rmse = math.sqrt(np.mean(errors_sq))
        self._record('RMSE vs GT [m]', rmse, KPI['rmse_bt_vs_gt_max_m'], '≤')
        return rmse

    def _check_ann_diagnostics(self, reader):
        # /ann/diagnostics Float32MultiArray layout:
        # [0] n_samples  [1] last_loss  [2] weight_norm
        # [3] is_trained [4] gps_gate   [5] in_std_min  [6] tgt_std_min
        max_samples, last_loss, min_std_seen = 0, float('nan'), float('inf')
        count = 0
        for ts, msg in iter_topic(reader, '/ann/diagnostics'):
            d = msg.data
            if len(d) < 7:
                continue
            max_samples = max(max_samples, int(d[0]))
            last_loss   = float(d[1])
            min_std_seen = min(min_std_seen, float(d[5]), float(d[6]))
            count += 1

        if count == 0:
            print('  ⚠  /ann/diagnostics not in bag — update ANN.py and rebuild')
            self._record('ANN samples (buf)', 0,           KPI['ann_samples_min'], '≥')
            self._record('ANN last loss',     float('nan'), KPI['ann_loss_max'],    '≤')
        else:
            self._record('ANN samples (buf)', max_samples, KPI['ann_samples_min'], '≥')
            self._record('ANN last loss',     last_loss,   KPI['ann_loss_max'],    '≤')
            if min_std_seen < 1e-4:
                print(f'  ⚠  in_std or tgt_std near zero ({min_std_seen:.2e}) '
                      f'— B4 guard may be insufficient at current robot speed')
        return max_samples, last_loss

    def _check_waypoints_from_log(self) -> int:
        if not os.path.exists(self.log_path):
            print(f'  ⚠  Launch log not found: {self.log_path}')
            self._record('WPs reached', 0, KPI['waypoints_required'], '≥')
            return 0
        with open(self.log_path) as f:
            log = f.read()
        # Matches: "Waypoint 47 reached" or "Waypoint 47/92 reached"
        indices = re.findall(r'Waypoint\s+(\d+)(?:/\d+)?\s+reached', log)
        count = len(set(indices))
        self._record('WPs reached', count, KPI['waypoints_required'], '≥')
        return count

    def _check_ann_samples_from_log(self) -> int:
        if not os.path.exists(self.log_path):
            return 0
        with open(self.log_path) as f:
            log = f.read()
        matches = re.findall(r'Training buffer:\s*(\d+)', log)
        return max((int(m) for m in matches), default=0)

    def _check_geofence_timing(self, reader):
        GATE_Y = -4.00  # building south boundary in /odom frame
        odom_buf = []
        for ts, msg in iter_topic(reader, '/odom'):
            odom_buf.append((ts, msg.pose.pose.position.y))

        transitions = []
        last_state = None
        for ts, msg in iter_topic(reader, '/bt/indoor_detection'):
            state = msg.data
            if last_state is not None and state != last_state:
                # Find /odom y at this timestamp
                closest_y = min(odom_buf, key=lambda o: abs(o[0] - ts))[1] \
                    if odom_buf else float('nan')
                transitions.append((ts / 1e9, last_state, state, closest_y))
            last_state = state

        print('\n  GEOFENCE transition log:')
        if not transitions:
            print('    (none detected)')
        for t, frm, to, y in transitions:
            err = abs(y - GATE_Y)
            flag = '✅' if err < 0.30 else '⚠ '
            print(f'    {flag}  t={t:7.1f}s  {frm:8s}→{to:8s}  '
                  f'/odom y={y:+.3f}m  Δ={err:.3f}m from gate')

    def run(self) -> bool:
        print('\n' + '═' * 58)
        print('  HYBRID LOCALIZATION — RUN VALIDATION')
        print('═' * 58)
        print(f'  Bag : {self.bag_path}')
        print(f'  Log : {self.log_path}')
        print('═' * 58 + '\n')

        wps     = self._check_waypoints_from_log()

        with Reader(self.bag_path) as reader:

            # Show available topics
            available = {c.topic for c in reader.connections}
            missing_topics = {
                '/ann/diagnostics', '/gazebo/model_states', '/odometry/global2'
            } - available
            if missing_topics:
                print(f'  ℹ  Topics not in bag (partial coverage): '
                      f'{", ".join(sorted(missing_topics))}\n')

            spin    = self._check_spin_events(reader)
            gps     = self._check_gps_offset(reader)
            ann_v   = self._check_ann_validity(reader)
            cov     = self._check_ekf_covariance(reader)
            dur     = self._check_run_time(reader)
            trans   = self._check_transitions(reader)
            rmse    = self._check_rmse_vs_gt(reader)
            s, loss = self._check_ann_diagnostics(reader)

            self._check_geofence_timing(reader)

        if gps:
            print(f'\n  GPS offset stats — mean={gps["mean"]:.3f}m  '
                  f'max={gps["max"]:.3f}m  std={gps["std"]:.3f}m')

        print('\n' + '─' * 58)
        print(f'  {"CHECK":<30} {"VALUE":>10}   {"LIMIT":>8}')
        print('─' * 58)

        for name, (val, limit, op, status) in self.results.items():
            if isinstance(val, float) and (math.isnan(val) or math.isinf(val)):
                val_str = '   NO DATA'
            elif isinstance(val, float):
                val_str = f'{val:>10.3f}'
            else:
                val_str = f'{val:>10d}'
            lim_str = f'{op} {limit}'
            print(f'  {status}  {name:<30} {val_str}   {lim_str}')

        print('─' * 58)
        verdict = ('🟢  ALL KPIs PASS'
                   if self._all_pass else
                   '🔴  FAILURES DETECTED — see table above')
        print(f'\n  {verdict}')
        print('═' * 58 + '\n')
        return self._all_pass


def main():
    ap = argparse.ArgumentParser(
        description='Offline validation: rosbag + launch log → KPI report')
    ap.add_argument('--bag', default=None,
                    help='Path to bag directory (auto-detects latest if omitted)')
    ap.add_argument('--log', default=DEFAULT_LOG,
                    help=f'Path to launch log (default: {DEFAULT_LOG})')
    args = ap.parse_args()

    bag_path = args.bag if args.bag else find_latest_bag(DEFAULT_BAGS_DIR)
    log_path = args.log

    if not os.path.exists(bag_path):
        print(f'ERROR: bag path not found: {bag_path}', file=sys.stderr)
        sys.exit(1)

    ok = RunValidator(bag_path, log_path).run()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()