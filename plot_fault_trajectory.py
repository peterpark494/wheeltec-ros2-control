#!/usr/bin/env python3
"""Plot one fault-tolerant straight-line run against its reference line."""

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
    parser.add_argument('--output', required=True, type=Path,
                        help='Output PNG path')
    parser.add_argument('--reference-length', type=float, default=3.6)
    parser.add_argument(
        '--tools-dir', type=Path,
        default=Path(__file__).resolve().parent /
        'ftag_waypoint_controller' / 'tools',
        help='Directory containing plot_straight_line.py')
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

    figure, axis = plt.subplots(figsize=(12, 6.2), dpi=180)
    axis.plot(
        [0.0, args.reference_length], [0.0, 0.0],
        '--', color='#777777', linewidth=1.8, label='Reference')
    axis.plot(
        actual_x, actual_y,
        color='#2478B5', linewidth=2.8, label='FTGA FTC trajectory')
    axis.scatter(
        [0.0], [0.0], s=55, color='black', zorder=5, label='Start')

    axis.set_title(
        'Straight-line Fault-Reconfiguration Experiment', fontsize=16)
    axis.set_xlabel('Longitudinal displacement (m)', fontsize=12)
    axis.set_ylabel('Lateral displacement (m)', fontsize=12)
    axis.set_xlim(-0.03, args.reference_length + 0.08)
    axis.set_ylim(-0.4, 1.0)
    axis.set_yticks(np.arange(-0.4, 1.01, 0.2))
    axis.grid(True, color='#dddddd', linewidth=0.8, alpha=0.85)
    axis.legend(loc='upper right', fontsize=10)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    figure.tight_layout()
    figure.savefig(args.output, bbox_inches='tight')
    plt.close(figure)
    print(f'Created {args.output}')


if __name__ == '__main__':
    main()
