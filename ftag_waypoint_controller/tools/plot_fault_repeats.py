#!/usr/bin/env python3
"""Plot repeated fault-reconfiguration runs against one straight reference."""

import argparse
from pathlib import Path

import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from plot_straight_line import process_run, run_metrics, write_csv  # noqa: E402


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--bags', required=True, nargs='+', type=Path)
    parser.add_argument('--output-dir', required=True, type=Path)
    parser.add_argument('--reference-length', type=float, default=3.6)
    args = parser.parse_args()
    if len(args.bags) < 2:
        raise ValueError('At least two repeated fault bags are required')
    args.output_dir.mkdir(parents=True, exist_ok=True)

    runs = [
        process_run(path, f'Fault run {index}')
        for index, path in enumerate(args.bags, start=1)
    ]
    errors = [float(np.max(np.abs(run.lateral))) for run in runs]
    worst_index = int(np.argmax(errors))

    figure, axis = plt.subplots(figsize=(9.0, 4.8))
    axis.plot(
        [0.0, args.reference_length], [0.0, 0.0], '--', color='0.40',
        linewidth=1.6, label='Reference')
    colors = ['#4C78A8', '#F58518', '#54A24B', '#B279A2', '#72B7B2']
    for index, run in enumerate(runs):
        worst = index == worst_index
        label = f'Fault run {index + 1}'
        if worst:
            label += ' (largest error)'
        axis.plot(
            run.longitudinal, run.lateral,
            color=colors[index % len(colors)],
            linewidth=3.2 if worst else 1.5,
            alpha=1.0 if worst else 0.82,
            zorder=4 if worst else 2,
            label=label)
        write_csv(run, args.output_dir / f'fault_run_{index + 1:02d}.csv')

    maximum_x = max(
        args.reference_length,
        *(float(np.max(run.longitudinal)) for run in runs),
    )
    axis.scatter([0.0], [0.0], color='black', marker='o', s=24,
                 zorder=5, label='Start')
    axis.set_xlim(-0.03, maximum_x + 0.08)
    axis.set_ylim(-0.4, 1.0)
    axis.set_yticks(np.arange(-0.4, 1.01, 0.2))
    axis.set_xlabel('Longitudinal displacement (m)')
    axis.set_ylabel('Lateral displacement (m)')
    axis.set_title('Repeated Fault-Reconfiguration Trajectories')
    axis.grid(True, color='0.90', linewidth=0.8)
    axis.legend(loc='best', frameon=True)
    figure.tight_layout()
    figure.savefig(
        args.output_dir / 'fault_repeated_trajectories.png',
        dpi=300, bbox_inches='tight')
    plt.close(figure)

    worst_run = runs[worst_index]
    figure, axis = plt.subplots(figsize=(9.0, 4.8))
    axis.plot(
        [0.0, args.reference_length], [0.0, 0.0], '--', color='0.40',
        linewidth=1.6, label='Reference')
    axis.plot(
        worst_run.longitudinal, worst_run.lateral,
        color='#1f77b4', linewidth=2.6,
        label=f'Fault trajectory (run {worst_index + 1})')
    axis.scatter([0.0], [0.0], color='black', marker='o', s=24,
                 zorder=5, label='Start')
    axis.set_xlim(-0.03, max(
        args.reference_length,
        float(np.max(worst_run.longitudinal))) + 0.08)
    axis.set_ylim(-0.4, 1.0)
    axis.set_yticks(np.arange(-0.4, 1.01, 0.2))
    axis.set_xlabel('Longitudinal displacement (m)')
    axis.set_ylabel('Lateral displacement (m)')
    axis.set_title('Straight-line Fault-Reconfiguration Experiment')
    axis.grid(True, color='0.90', linewidth=0.8)
    axis.legend(loc='best', frameon=True)
    figure.tight_layout()
    figure.savefig(
        args.output_dir / 'fault_largest_error_trajectory.png',
        dpi=300, bbox_inches='tight')
    plt.close(figure)

    with (args.output_dir / 'fault_repeated_metrics.txt').open(
            'w', encoding='utf-8') as stream:
        for index, run in enumerate(runs):
            stream.write(f'[Fault run {index + 1}]\n')
            for name, value in run_metrics(run).items():
                if isinstance(value, float):
                    stream.write(f'{name}: {value:.6f}\n')
                else:
                    stream.write(f'{name}: {value}\n')
            stream.write(
                f'is_largest_error: {str(index == worst_index).lower()}\n\n')

    print(f'Largest-error run: {worst_index + 1} '
          f'({errors[worst_index]:.6f} m)')
    print(f'Created results in {args.output_dir}')


if __name__ == '__main__':
    main()
