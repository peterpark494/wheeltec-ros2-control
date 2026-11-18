"""Track an odometry-relative quadratic reference curve."""

import math

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

from .curve_guidance import (
    quadratic_cross_track,
    quadratic_curve_waypoints,
    start_frame_position,
)
from .line_guidance import body_errors, paper_velocity_command
from .odom_line_node import yaw_from_quaternion


class OdomCurveController(Node):
    """Paper kinematic outer loop on ``y = a*x^2`` in the start frame."""

    def __init__(self):
        super().__init__('odom_curve_controller')
        defaults = {
            'odom_topic': '/odom',
            'cmd_vel_topic': '/cmd_vel',
            'x_length_m': 3.20,
            'curve_coefficient': 0.05,
            'waypoint_spacing_m': 0.10,
            'acceptance_radius_m': 0.16,
            'end_tolerance_m': 0.10,
            'max_linear_mps': 0.15,
            'max_angular_rps': 0.25,
            'linear_gain': 2.0,
            'angular_gain': 1.5,
            'max_cross_track_m': 0.80,
            'odom_timeout_s': 0.50,
            'max_run_time_s': 90.0,
            'control_rate_hz': 20.0,
            'enabled': False,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        def get(name):
            return self.get_parameter(name).value

        self.x_length = float(get('x_length_m'))
        self.coefficient = float(get('curve_coefficient'))
        self.spacing = float(get('waypoint_spacing_m'))
        self.acceptance_radius = float(get('acceptance_radius_m'))
        self.end_tolerance = float(get('end_tolerance_m'))
        self.max_linear = float(get('max_linear_mps'))
        self.max_angular = float(get('max_angular_rps'))
        self.linear_gain = float(get('linear_gain'))
        self.angular_gain = float(get('angular_gain'))
        self.max_cross = float(get('max_cross_track_m'))
        self.odom_timeout = float(get('odom_timeout_s'))
        self.max_run_time = float(get('max_run_time_s'))
        rate = float(get('control_rate_hz'))
        positive = (
            self.x_length, self.spacing, self.acceptance_radius,
            self.end_tolerance, self.max_linear, self.max_angular,
            self.linear_gain, self.angular_gain, self.max_cross,
            self.odom_timeout, self.max_run_time, rate,
        )
        if not all(math.isfinite(value) and value > 0.0
                   for value in positive):
            raise ValueError('Curve-controller limits must be positive')
        if not math.isfinite(self.coefficient) or self.coefficient < 0.0:
            raise ValueError('Curve coefficient must be finite and nonnegative')

        self.publisher = self.create_publisher(
            Twist, str(get('cmd_vel_topic')), 10)
        self.subscription = self.create_subscription(
            Odometry, str(get('odom_topic')), self.on_odom, 10)
        self.timer = self.create_timer(1.0 / rate, self.on_timer)
        self.pose = None
        self.odom_time = None
        self.was_enabled = False
        self.finished = False
        self.start_time = None
        self.start_pose = None
        self.waypoints = []
        self.waypoint_index = 0
        self.previous_error = None
        self.last_warning_ns = 0
        self.get_logger().info(
            f'Ready for local y={self.coefficient:.3f}*x^2, '
            f'0<=x<={self.x_length:.2f}m. Set enabled=true to capture '
            'the current odom pose and move.')

    def on_odom(self, msg):
        """Store the latest finite odometry pose."""
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        pose = (
            float(position.x), float(position.y),
            yaw_from_quaternion(orientation),
        )
        if all(math.isfinite(value) for value in pose):
            self.pose = pose
            self.odom_time = self.get_clock().now()

    def stop(self):
        """Publish a zero body-velocity command."""
        self.publisher.publish(Twist())

    def reset(self):
        """Reset the captured curve and experiment state."""
        self.finished = False
        self.start_time = None
        self.start_pose = None
        self.waypoints = []
        self.waypoint_index = 0
        self.previous_error = None

    def warn_periodically(self, message):
        """Limit repeated warning output."""
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self.last_warning_ns >= 2_000_000_000:
            self.get_logger().warn(message)
            self.last_warning_ns = now_ns

    def initialize_curve(self):
        """Capture the current pose and build the reference waypoints."""
        self.start_pose = self.pose
        start_x, start_y, start_yaw = self.start_pose
        self.waypoints = quadratic_curve_waypoints(
            start_x, start_y, start_yaw, self.x_length,
            self.coefficient, self.spacing)
        self.waypoint_index = 0
        self.start_time = self.get_clock().now()
        end_x, end_y, end_yaw, _ = self.waypoints[-1]
        self.get_logger().info(
            f'Captured curve start=({start_x:.3f}, {start_y:.3f}), '
            f'yaw={start_yaw:.3f}; end=({end_x:.3f}, {end_y:.3f}), '
            f'end_yaw={end_yaw:.3f}; waypoints={len(self.waypoints)}.')

    def finish(self, message):
        """Latch a completed or aborted run and stop the chassis."""
        self.finished = True
        self.stop()
        self.get_logger().info(message)

    def on_timer(self):
        """Run one guarded curve-tracking update."""
        enabled = bool(self.get_parameter('enabled').value)
        if not enabled:
            if self.was_enabled:
                self.stop()
                self.get_logger().info('Disabled; stopped and reset.')
            self.was_enabled = False
            self.reset()
            return
        self.was_enabled = True

        if self.pose is None or self.odom_time is None:
            self.stop()
            self.warn_periodically('No odometry received; stopped.')
            return
        age = (self.get_clock().now() - self.odom_time).nanoseconds / 1e9
        if age < 0.0 or age > self.odom_timeout:
            self.stop()
            self.warn_periodically(f'Odometry is stale ({age:.2f}s); stopped.')
            return
        if self.finished:
            self.stop()
            return
        if self.start_pose is None:
            self.initialize_curve()

        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        if elapsed < 0.0 or elapsed > self.max_run_time:
            self.finish('Maximum run time reached; stopped.')
            return

        x, y, yaw = self.pose
        start_x, start_y, start_yaw = self.start_pose
        local_x, local_y = start_frame_position(
            x, y, start_x, start_y, start_yaw)
        cross = quadratic_cross_track(
            local_x, local_y, self.coefficient)
        if abs(cross) > self.max_cross:
            self.finish(
                f'Curve error {cross:.2f}m exceeds '
                f'{self.max_cross:.2f}m; stopped.')
            return

        while self.waypoint_index < len(self.waypoints):
            target = self.waypoints[self.waypoint_index]
            distance = math.hypot(target[0] - x, target[1] - y)
            radius = (self.end_tolerance
                      if self.waypoint_index == len(self.waypoints) - 1
                      else self.acceptance_radius)
            # Advancing after passing a local-x station prevents one missed
            # intermediate radius from permanently stalling the experiment.
            if distance > radius and local_x < target[3]:
                break
            self.waypoint_index += 1
            self.previous_error = None

        if self.waypoint_index == len(self.waypoints):
            self.finish(
                f'Curve complete: local_x={local_x:.3f}m, '
                f'cross={cross:.3f}m; stopped.')
            return

        target_x, target_y, target_yaw, _ = (
            self.waypoints[self.waypoint_index])
        current_error = body_errors(
            x, y, yaw, target_x, target_y, target_yaw)
        if self.previous_error is None:
            self.previous_error = current_error
        linear, angular = paper_velocity_command(
            self.previous_error[0], self.previous_error[2],
            self.max_linear, self.max_angular,
            self.linear_gain, self.angular_gain)
        self.previous_error = current_error
        command = Twist()
        command.linear.x = float(linear)
        command.angular.z = float(angular)
        self.publisher.publish(command)

    def destroy_node(self):
        if self.was_enabled and rclpy.ok():
            self.stop()
        return super().destroy_node()


def main(args=None):
    """Run the odometry-relative quadratic curve controller."""
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    try:
        node = OdomCurveController()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
