#!/usr/bin/env python3
"""Create straight-line trajectory plots and CSV files from ROS 2 bags."""

import argparse
import csv
from dataclasses import dataclass
import math
from pathlib import Path

import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import rosbag2_py  # noqa: E402
from rclpy.serialization import deserialize_message  # noqa: E402
from rosidl_runtime_py.utilities import get_message  # noqa: E402


@dataclass
class RunData:
    """Processed samples from one active straight-line run."""

    label: str
    time_s: np.ndarray
    odom_x: np.ndarray
    odom_y: np.ndarray
    longitudinal: np.ndarray
    lateral: np.ndarray
    yaw: np.ndarray
    effectiveness: np.ndarray
    nominal_linear: np.ndarray
    nominal_angular: np.ndarray
    allocated_linear: np.ndarray
    allocated_angular: np.ndarray
    output_linear: np.ndarray
    output_angular: np.ndarray


def quaternion_yaw(quaternion):
    """Return planar yaw from a ROS quaternion."""
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z
               + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y
                     + quaternion.z * quaternion.z),
    )


def read_messages(bag_path):
    """Read the four experiment topics from a rosbag2 directory."""
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id='sqlite3'),
        rosbag2_py.ConverterOptions('', ''),
    )
    topic_types = {
        item.name: item.type for item in reader.get_all_topics_and_types()
    }
    wanted = {
        '/odom', '/cmd_vel_nominal', '/cmd_vel_allocated', '/cmd_vel',
        '/fault/effectiveness',
    }
    if '/odom' not in topic_types or '/cmd_vel' not in topic_types:
        raise RuntimeError(f'{bag_path} must contain /odom and /cmd_vel')
    wanted.intersection_update(topic_types)

    messages = {name: [] for name in wanted}
    message_types = {
        name: get_message(topic_types[name]) for name in wanted
    }
    while reader.has_next():
        topic, serialized, timestamp = reader.read_next()
        if topic in wanted:
            message = deserialize_message(serialized, message_types[topic])
            messages[topic].append((timestamp, message))
    return messages


def movement_window(commands, threshold=1e-5):
    """Find the first continuous motion interval in nominal commands."""
    start = None
    stop = None
    for timestamp, message in commands:
        moving = (abs(float(message.linear.x)) > threshold or
                  abs(float(message.angular.z)) > threshold)
        if start is None and moving:
            start = timestamp
        elif start is not None and not moving:
            stop = timestamp
            break
    if start is None:
        raise RuntimeError('No nonzero nominal command was found')
    if stop is None:
        stop = commands[-1][0]
    return start, stop


def interpolate_scalar(samples, timestamps, value_getter, default):
    """Interpolate a scalar topic onto odometry timestamps."""
    if not samples:
        return np.full(len(timestamps), default, dtype=float)
    source_time = np.asarray([item[0] for item in samples], dtype=float)
    source_value = np.asarray(
        [value_getter(item[1]) for item in samples], dtype=float)
    return np.interp(
        np.asarray(timestamps, dtype=float), source_time, source_value,
        left=source_value[0], right=source_value[-1])


def process_run(bag_path, label):
    """Convert raw odometry into the captured start-pose coordinate frame."""
    messages = read_messages(bag_path)
    nominal_source = messages.get('/cmd_vel_nominal', messages['/cmd_vel'])
    start, stop = movement_window(nominal_source)
    odometry = [
        item for item in messages['/odom'] if start <= item[0] <= stop
    ]
    if len(odometry) < 2:
        raise RuntimeError(f'Not enough active odometry samples in {bag_path}')

    timestamps = np.asarray([item[0] for item in odometry], dtype=np.int64)
    x = np.asarray([
        float(item[1].pose.pose.position.x) for item in odometry
    ])
    y = np.asarray([
        float(item[1].pose.pose.position.y) for item in odometry
    ])
    yaw = np.asarray([
        quaternion_yaw(item[1].pose.pose.orientation) for item in odometry
    ])
    x0 = x[0]
    y0 = y[0]
    yaw0 = yaw[0]
    dx = x - x0
    dy = y - y0
    longitudinal = math.cos(yaw0) * dx + math.sin(yaw0) * dy
    # Use the paper figure's display convention: displacement to the
    # robot's right is negative, so the injected deviation bends downward.
    lateral = math.sin(yaw0) * dx - math.cos(yaw0) * dy

    active_effectiveness = [
        item for item in messages.get('/fault/effectiveness', [])
        if start <= item[0] <= stop
    ]
    active_nominal = [
        item for item in nominal_source
        if start <= item[0] <= stop
    ]
    active_output = [
        item for item in messages['/cmd_vel']
        if start <= item[0] <= stop
    ]
    active_allocated = [
        item for item in messages.get('/cmd_vel_allocated', [])
        if start <= item[0] <= stop
    ]
    effectiveness = interpolate_scalar(
        active_effectiveness, timestamps, lambda msg: msg.data, 1.0)
    nominal_linear = interpolate_scalar(
        active_nominal, timestamps, lambda msg: msg.linear.x, 0.0)
    nominal_angular = interpolate_scalar(
        active_nominal, timestamps, lambda msg: msg.angular.z, 0.0)
    if active_allocated:
        allocated_linear = interpolate_scalar(
            active_allocated, timestamps, lambda msg: msg.linear.x, 0.0)
        allocated_angular = interpolate_scalar(
            active_allocated, timestamps, lambda msg: msg.angular.z, 0.0)
    else:
        allocated_linear = nominal_linear.copy()
        allocated_angular = nominal_angular.copy()
    output_linear = interpolate_scalar(
        active_output, timestamps, lambda msg: msg.linear.x, 0.0)
    output_angular = interpolate_scalar(
        active_output, timestamps, lambda msg: msg.angular.z, 0.0)

    return RunData(
        label=label,
        time_s=(timestamps - timestamps[0]) / 1e9,
        odom_x=x,
        odom_y=y,
        longitudinal=longitudinal,
        lateral=lateral,
        yaw=np.unwrap(yaw),
        effectiveness=effectiveness,
        nominal_linear=nominal_linear,
        nominal_angular=nominal_angular,
        allocated_linear=allocated_linear,
        allocated_angular=allocated_angular,
        output_linear=output_linear,
        output_angular=output_angular,
    )


def write_csv(run, output_path):
    """Write processed positions and commands for later analysis."""
    columns = zip(
        run.time_s, run.odom_x, run.odom_y,
        run.longitudinal, run.lateral, run.yaw,
        run.effectiveness, run.nominal_linear, run.nominal_angular,
        run.allocated_linear, run.allocated_angular,
        run.output_linear, run.output_angular,
    )
    with output_path.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.writer(stream)
        writer.writerow([
            'time_s', 'odom_x_m', 'odom_y_m', 'longitudinal_m',
            'lateral_m', 'yaw_rad', 'lambda_1',
            'nominal_linear_mps', 'nominal_angular_rps',
            'allocated_linear_mps', 'allocated_angular_rps',
            'output_linear_mps', 'output_angular_rps',
        ])
        writer.writerows(columns)


def run_metrics(run):
    """Return concise trajectory statistics."""
    return {
        'duration_s': float(run.time_s[-1]),
        'samples': int(len(run.time_s)),
        'final_longitudinal_m': float(run.longitudinal[-1]),
        'final_lateral_m': float(run.lateral[-1]),
        'max_abs_lateral_m': float(np.max(np.abs(run.lateral))),
        'lateral_rmse_m': float(np.sqrt(np.mean(run.lateral ** 2))),
        'minimum_lambda_1': float(np.min(run.effectiveness)),
    }


def save_metrics(runs, output_path):
    """Write human-readable statistics for all runs."""
    with output_path.open('w', encoding='utf-8') as stream:
        for run in runs:
            stream.write(f'[{run.label}]\n')
            for name, value in run_metrics(run).items():
                if isinstance(value, float):
                    stream.write(f'{name}: {value:.6f}\n')
                else:
                    stream.write(f'{name}: {value}\n')
            stream.write('\n')


def plot_trajectories(healthy, fault, reference_length, output_path):
    """Plot the reference, healthy run, and fault run together."""
    figure, axis = plt.subplots(figsize=(9.0, 4.8))
    axis.plot(
        [0.0, reference_length], [0.0, 0.0], '--', color='0.45',
        linewidth=1.5, label='Reference')
    axis.plot(
        healthy.longitudinal, healthy.lateral,
        color='#2ca02c', linewidth=1.8, label='Healthy trajectory')
    axis.plot(
        fault.longitudinal, fault.lateral,
        color='#1f77b4', linewidth=1.8,
        label='Fault + velocity reconfiguration')
    axis.scatter([0.0], [0.0], color='black', marker='o', s=24,
                 zorder=5, label='Start')
    maximum_x = max(
        reference_length,
        float(np.max(healthy.longitudinal)),
        float(np.max(fault.longitudinal)),
    )
    axis.set_xlim(-0.03, maximum_x + 0.08)
    axis.set_ylim(-0.4, 1.0)
    axis.set_yticks(np.arange(-0.4, 1.01, 0.2))
    axis.set_xlabel('Longitudinal displacement (m)')
    axis.set_ylabel('Lateral displacement (m)')
    axis.set_title('Straight-line Trajectory Tracking Experiment')
    axis.grid(True, color='0.90', linewidth=0.8)
    axis.legend(loc='best', frameon=True)
    figure.tight_layout()
    figure.savefig(output_path, dpi=300, bbox_inches='tight')
    plt.close(figure)


def plot_fault_effectiveness(fault, output_path):
    """Plot the injected front-left effectiveness over active time."""
    figure, axis = plt.subplots(figsize=(8.0, 4.2))
    axis.plot(
        fault.time_s, fault.effectiveness,
        color='#d62728', linewidth=1.8)
    axis.set_xlabel('Time (s)')
    axis.set_ylabel(r'Front-left effectiveness $\lambda_1$')
    axis.set_title('Injected Actuator Effectiveness')
    axis.set_ylim(0.65, 1.02)
    axis.grid(True, color='0.90', linewidth=0.8)
    figure.tight_layout()
    figure.savefig(output_path, dpi=300, bbox_inches='tight')
    plt.close(figure)


def main():
    """Parse command-line arguments and generate all artifacts."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--healthy-bag', required=True, type=Path)
    parser.add_argument('--fault-bag', required=True, type=Path)
    parser.add_argument('--output-dir', required=True, type=Path)
    parser.add_argument('--reference-length', type=float, default=3.6)
    arguments = parser.parse_args()
    if arguments.reference_length <= 0.0:
        raise ValueError('Reference length must be positive')
    arguments.output_dir.mkdir(parents=True, exist_ok=True)

    healthy = process_run(arguments.healthy_bag, 'Healthy')
    fault = process_run(arguments.fault_bag, 'Front-left fault')
    write_csv(healthy, arguments.output_dir / 'healthy_trajectory.csv')
    write_csv(fault, arguments.output_dir / 'fault_trajectory.csv')
    save_metrics(
        [healthy, fault], arguments.output_dir / 'trajectory_metrics.txt')
    plot_trajectories(
        healthy, fault, arguments.reference_length,
        arguments.output_dir / 'trajectory_comparison.png')
    plot_fault_effectiveness(
        fault, arguments.output_dir / 'fault_effectiveness.png')
    print(f'Created results in {arguments.output_dir}')


if __name__ == '__main__':
    main()
