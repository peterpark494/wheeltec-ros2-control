"""Track collinear map waypoints with the paper's kinematic outer loop."""

import math

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from .line_guidance import (
    body_errors,
    collinear_waypoints,
    line_errors,
    paper_velocity_command,
    waypoint_complete,
    wrap_angle,
)


def yaw_from_quaternion(q):
    """Extract planar yaw from a geometry_msgs quaternion."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class MapLineController(Node):
    """Paper equations (10), (15), and (18) on a fixed map-frame line."""

    def __init__(self):
        super().__init__('map_line_controller')
        defaults = {
            'map_frame': 'map',
            'base_frame': 'base_link',
            'cmd_vel_topic': '/cmd_vel',
            'start_x': -0.05283617973327637,
            'start_y': 0.005418464541143524,
            'end_x': 1.9724102020263672,
            'end_y': -0.0378357470035553,
            'max_linear_mps': 0.08,
            'max_angular_rps': 0.35,
            'linear_gain': 2.0,
            'angular_gain': 1.5,
            'waypoint_spacing_m': 0.40,
            'acceptance_radius_m': 0.20,
            'start_tolerance_m': 0.10,
            'start_yaw_tolerance_rad': 0.20,
            'end_tolerance_m': 0.10,
            'max_cross_track_m': 0.40,
            'tf_timeout_s': 1.0,
            'max_run_time_s': 90.0,
            'control_rate_hz': 20.0,
            'enabled': False,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        def get(name):
            return self.get_parameter(name).value

        self.map_frame = str(get('map_frame'))
        self.base_frame = str(get('base_frame'))
        self.start_x = float(get('start_x'))
        self.start_y = float(get('start_y'))
        self.end_x = float(get('end_x'))
        self.end_y = float(get('end_y'))
        self.max_linear = float(get('max_linear_mps'))
        self.max_angular = float(get('max_angular_rps'))
        self.linear_gain = float(get('linear_gain'))
        self.angular_gain = float(get('angular_gain'))
        self.waypoint_spacing = float(get('waypoint_spacing_m'))
        self.acceptance_radius = float(get('acceptance_radius_m'))
        self.start_tolerance = float(get('start_tolerance_m'))
        self.start_yaw_tolerance = float(get('start_yaw_tolerance_rad'))
        self.end_tolerance = float(get('end_tolerance_m'))
        self.max_cross = float(get('max_cross_track_m'))
        self.tf_timeout = float(get('tf_timeout_s'))
        self.max_run_time = float(get('max_run_time_s'))
        rate = float(get('control_rate_hz'))

        _, _, self.line_yaw, self.line_length = line_errors(
            self.start_x, self.start_y, self.start_x, self.start_y,
            self.end_x, self.end_y)
        self.waypoints = collinear_waypoints(
            self.start_x, self.start_y, self.end_x, self.end_y,
            self.waypoint_spacing)
        positive = (self.max_linear, self.max_angular, self.linear_gain,
                    self.angular_gain, self.waypoint_spacing,
                    self.acceptance_radius,
                    self.start_tolerance, self.start_yaw_tolerance,
                    self.end_tolerance,
                    self.max_cross, self.tf_timeout, self.max_run_time, rate)
        if not all(math.isfinite(value) and value > 0.0 for value in positive):
            raise ValueError('Controller limits, gains, tolerances and rate must be positive')
        if not self.map_frame or not self.base_frame:
            raise ValueError('Frame names must not be empty')
        if self.map_frame == self.base_frame:
            raise ValueError('map_frame and base_frame must be different')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.publisher = self.create_publisher(
            Twist, str(get('cmd_vel_topic')), 10)
        self.timer = self.create_timer(1.0 / rate, self.on_timer)
        self.was_enabled = False
        self.started = False
        self.finished = False
        self.waypoint_index = 0
        self.start_time = None
        self.previous_error = None
        self.last_warning_ns = 0
        self.get_logger().info(
            f'Paper outer loop ready: line={self.line_length:.3f}m, '
            f'waypoints={len(self.waypoints)}, yaw={self.line_yaw:.3f}rad. '
            'Set enabled=true to move.')

    def stop(self):
        """Publish an all-zero velocity command."""
        self.publisher.publish(Twist())

    def warn_periodically(self, message):
        """Limit repeated safety warnings to one every two seconds."""
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self.last_warning_ns >= 2_000_000_000:
            self.get_logger().warn(message)
            self.last_warning_ns = now_ns

    def reset(self):
        """Reset route state after disabling the controller."""
        self.started = False
        self.finished = False
        self.waypoint_index = 0
        self.start_time = None
        self.previous_error = None

    def lookup_pose(self):
        """Get the latest finite and sufficiently fresh map-frame pose."""
        transform = self.tf_buffer.lookup_transform(
            self.map_frame, self.base_frame, Time())
        now = self.get_clock().now()
        stamp = Time.from_msg(transform.header.stamp)
        age = (now - stamp).nanoseconds / 1e9
        # Localization may future-date map->odom by a configured tolerance.
        if age < -2.0 or age > self.tf_timeout:
            raise TransformException(f'transform age is {age:.2f}s')
        x = float(transform.transform.translation.x)
        y = float(transform.transform.translation.y)
        yaw = yaw_from_quaternion(transform.transform.rotation)
        if not all(math.isfinite(value) for value in (x, y, yaw)):
            raise TransformException('map pose contains non-finite values')
        return x, y, yaw

    def on_timer(self):
        """Run one guarded velocity-control iteration."""
        enabled = bool(self.get_parameter('enabled').value)
        if not enabled:
            if self.was_enabled:
                self.stop()
                self.get_logger().info('Disabled; stopped and reset.')
            self.was_enabled = False
            self.reset()
            return
        self.was_enabled = True

        try:
            x, y, yaw = self.lookup_pose()
        except TransformException as error:
            self.stop()
            self.warn_periodically(f'No usable map pose: {error}; stopped.')
            return

        if self.finished:
            self.stop()
            return

        along, cross, _, length = line_errors(
            x, y, self.start_x, self.start_y, self.end_x, self.end_y)

        if not self.started:
            distance = math.hypot(x - self.start_x, y - self.start_y)
            if distance > self.start_tolerance:
                self.stop()
                self.warn_periodically(
                    f'Robot is {distance:.2f}m from start; '
                    f'must be within {self.start_tolerance:.2f}m.')
                return
            yaw_error = wrap_angle(self.line_yaw - yaw)
            if abs(yaw_error) > self.start_yaw_tolerance:
                self.stop()
                self.warn_periodically(
                    f'Start yaw error is {yaw_error:.2f}rad; must be within '
                    f'{self.start_yaw_tolerance:.2f}rad of line direction.')
                return
            self.started = True
            self.start_time = self.get_clock().now()
            self.get_logger().info('Fixed-line run started.')

        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        if elapsed < 0.0 or elapsed > self.max_run_time:
            self.finished = True
            self.stop()
            self.get_logger().warn('Maximum run time reached; stopped.')
            return

        if abs(cross) > self.max_cross:
            self.finished = True
            self.stop()
            self.get_logger().warn(
                f'Cross-track error {cross:.2f}m exceeds '
                f'{self.max_cross:.2f}m; stopped.')
            return

        if along > length + self.end_tolerance:
            self.finished = True
            self.stop()
            self.get_logger().warn('Line end was passed; stopped.')
            return

        while self.waypoint_index < len(self.waypoints):
            target_x, target_y = self.waypoints[self.waypoint_index]
            distance = math.hypot(target_x - x, target_y - y)
            radius = (self.end_tolerance
                      if self.waypoint_index == len(self.waypoints) - 1
                      else self.acceptance_radius)
            if not waypoint_complete(distance, radius):
                break
            self.waypoint_index += 1
            self.previous_error = None
            self.get_logger().info(
                f'Reached waypoint {self.waypoint_index}/'
                f'{len(self.waypoints)}.')

        if self.waypoint_index == len(self.waypoints):
            self.finished = True
            self.stop()
            self.get_logger().info(
                f'Line complete: cross={cross:.3f}m; stopped.')
            return

        target_x, target_y = self.waypoints[self.waypoint_index]
        current_error = body_errors(
            x, y, yaw, target_x, target_y, self.line_yaw)
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
    """Run the fixed map-line controller."""
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    try:
        node = MapLineController()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
