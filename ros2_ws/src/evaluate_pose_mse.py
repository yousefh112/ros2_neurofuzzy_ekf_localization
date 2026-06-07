#!/usr/bin/env python3
import argparse
import glob
import os
import sys

import numpy as np
import pandas as pd


def parse_args():
    parser = argparse.ArgumentParser(
        description='Compute mean error and standard deviation for odom vs ann_trajectory and odom vs bt_fused.'
    )
    parser.add_argument(
        'files',
        nargs='*',
        help='CSV files to analyze. If omitted, the script will look for Test_*.csv in the current or parent directory.',
    )
    parser.add_argument(
        '--max-dt',
        type=float,
        default=0.05,
        help='Maximum allowed time gap in seconds for matching rows by timestamp.',
    )
    return parser.parse_args()


def find_default_files():
    cwd = os.getcwd()
    candidates = sorted(glob.glob(os.path.join(cwd, 'Test_*.csv')))
    if candidates:
        return candidates
    parent = os.path.dirname(cwd)
    candidates = sorted(glob.glob(os.path.join(parent, 'Test_*.csv')))
    return candidates


def load_csv(path):
    if not os.path.exists(path):
        raise FileNotFoundError(f'File not found: {path}')
    return pd.read_csv(path)


def match_by_time(reference, source, max_dt=0.05):
    if reference.empty or source.empty:
        return pd.DataFrame()

    ref = reference.sort_values('__time').reset_index(drop=True)
    src = source.sort_values('__time').reset_index(drop=True)
    ref_times = ref['__time'].to_numpy()
    src_times = src['__time'].to_numpy()
    idx = np.searchsorted(ref_times, src_times)
    matched_rows = []

    for i, t in enumerate(src_times):
        candidates = []
        if idx[i] > 0:
            candidates.append(idx[i] - 1)
        if idx[i] < len(ref_times):
            candidates.append(idx[i])
        if not candidates:
            continue
        best = min(candidates, key=lambda j: abs(ref_times[j] - t))
        dt = abs(ref_times[best] - t)
        if dt <= max_dt:
            matched_rows.append({
                'odom_x': ref.at[best, '/odom/pose/pose/position/x'],
                'odom_y': ref.at[best, '/odom/pose/pose/position/y'],
                'src_x': src.iloc[i, 1],
                'src_y': src.iloc[i, 2],
                'dt': dt,
            })

    return pd.DataFrame(matched_rows)


def compute_metrics(matched):
    if matched.empty:
        return None
    metrics = {}
    for axis in ['x', 'y']:
        diff = matched[f'odom_{axis}'] - matched[f'src_{axis}']
        metrics[axis] = {
            'mean_error': float(diff.mean()),
            'std': float(diff.std(ddof=0)),
            'n': int(diff.shape[0]),
        }
    metrics['dt_mean'] = float(matched['dt'].mean())
    metrics['dt_max'] = float(matched['dt'].max())
    return metrics


def compute_file_metrics(path, max_dt):
    df = load_csv(path)
    required = [
        '__time',
        '/odom/pose/pose/position/x',
        '/odom/pose/pose/position/y',
        '/ann/trajectory/x',
        '/ann/trajectory/y',
        '/odometry/bt_fused/pose/pose/position/x',
        '/odometry/bt_fused/pose/pose/position/y',
    ]
    missing = [c for c in required if c not in df.columns]
    if missing:
        raise ValueError(f'File {path} is missing required columns: {missing}')

    odom = df[['__time', '/odom/pose/pose/position/x', '/odom/pose/pose/position/y']].dropna()
    ann = df[['__time', '/ann/trajectory/x', '/ann/trajectory/y']].dropna()
    bt = df[['__time', '/odometry/bt_fused/pose/pose/position/x', '/odometry/bt_fused/pose/pose/position/y']].dropna()

    return {
        'file': os.path.basename(path),
        'ann': compute_metrics(match_by_time(odom, ann, max_dt=max_dt)),
        'bt_fused': compute_metrics(match_by_time(odom, bt, max_dt=max_dt)),
    }


def print_metrics(report, label):
    print(f'--- {label} ---')
    for source_name, data in [('ANN trajectory', report['ann']), ('bt_fused', report['bt_fused'])]:
        if data is None:
            print(f'{source_name}: no matched rows')
            continue
        print(source_name)
        print(f'  X: mean error={data["x"]["mean_error"]:.6g}, std={data["x"]["std"]:.6g}, n={data["x"]["n"]}')
        print(f'  Y: mean error={data["y"]["mean_error"]:.6g}, std={data["y"]["std"]:.6g}, n={data["y"]["n"]}')
        print(f'  matching dt mean={data["dt_mean"]:.6g}s max={data["dt_max"]:.6g}s')
        print()


def aggregate_reports(reports):
    def aggregate(source_key):
        x_err = []
        y_err = []
        for report in reports:
            data = report[source_key]
            if data is None:
                continue
            x_err.append((data['x']['mean_error'], data['x']['std'], data['x']['n']))
            y_err.append((data['y']['mean_error'], data['y']['std'], data['y']['n']))
        return x_err, y_err
    return {
        'ann': aggregate('ann'),
        'bt_fused': aggregate('bt_fused'),
    }


def main():
    args = parse_args()
    files = args.files or find_default_files()
    if not files:
        print('No CSV files provided and no Test_*.csv files found.', file=sys.stderr)
        return

    reports = []
    for path in files:
        try:
            reports.append(compute_file_metrics(path, args.max_dt))
        except Exception as e:
            print(f'ERROR processing {path}: {e}', file=sys.stderr)
            continue

    if not reports:
        print('No valid reports were generated.', file=sys.stderr)
        return

    print('Pose evaluation for odom vs ann_trajectory and odom vs bt_fused')
    print('Metric: mean error and standard deviation')
    print(f'Files: {files}')
    print(f'Max matching gap: {args.max_dt:.3f}s')
    print()

    for report in reports:
        print_metrics(report, report['file'])

    if len(reports) > 1:
        print('--- Combined report ---')
        print('Note: combined mean error/std is not computed automatically because files may have different counts. Use per-file results for reliable comparison.')


if __name__ == '__main__':
    main()
