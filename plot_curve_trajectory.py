#!/usr/bin/env python3
"""Plot one odometry-relative quadratic-curve tracking experiment."""

import argparse
import sys
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def parse_args():
    parser = argparse.ArgumentParser()
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument('--bag', type=Path, help='ROS 2 bag directory')
    source.add_argument('--points', type=Path, help='Tab-separated local trajectory points')
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--x-length', type=float, default=3.2)
    parser.add_argument('--coefficient', type=float, default=0.05)
    parser.add_argument(
        '--tools-dir', type=Path,
        default=Path(__file__).resolve().parent /
        'ftag_waypoint_controller' / 'tools')
    return parser.parse_args()


def main():
    args = parse_args()
    if args.points is not None:
        points = np.genfromtxt(args.points, delimiter='\t', names=True)
        actual_x, actual_y = points['longitudinal_m'], points['lateral_m']
    else:
        sys.path.insert(0, str(args.tools_dir))
        from plot_straight_line import process_run

        run = process_run(args.bag, 'FTGA FTC trajectory')
        actual_x, actual_y = run.longitudinal, -run.lateral
    reference_x = np.linspace(0.0, args.x_length, 500)
    reference_y = args.coefficient * reference_x ** 2

    figure, axis = plt.subplots(figsize=(12, 6.2), dpi=180)
    axis.plot(
        reference_x, reference_y, '--', color='#777777', linewidth=1.8,
        label='Reference')
    axis.plot(
        actual_x, actual_y, color='#2478B5', linewidth=2.6,
        label='FTGA FTC trajectory')
    axis.scatter(
        [0.0], [0.0], s=55, color='black', zorder=5, label='Start')

    axis.set_title('Curved-path Trajectory Tracking Experiment', fontsize=16)
    axis.set_xlabel('Longitudinal displacement (m)', fontsize=12)
    axis.set_ylabel('Lateral displacement (m)', fontsize=12)
    axis.set_xlim(-0.03, args.x_length + 0.08)
    axis.set_ylim(-0.05, 0.60)
    axis.set_xticks(np.arange(0.0, args.x_length + 0.01, 0.5))
    axis.set_yticks(np.arange(0.0, 0.61, 0.1))
    axis.grid(True, color='#dddddd', linewidth=0.8, alpha=0.85)
    axis.legend(loc='upper left', fontsize=10)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    figure.tight_layout()
    figure.savefig(args.output, bbox_inches='tight')
    plt.close(figure)
    print(f'Created {args.output}')


if __name__ == '__main__':
    main()
