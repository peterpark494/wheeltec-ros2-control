"""Track a line relative to the enable-time odometry pose."""

import math

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

from .line_guidance import (
    body_errors,
    collinear_waypoints,
    line_errors,
    paper_velocity_command,
    relative_line,
    waypoint_complete,
)


def yaw_from_quaternion(q):
    """Extract planar yaw from a geometry_msgs quaternion."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class OdomLineController(Node):
    """Paper kinematic outer loop on an enable-time odometry line."""

    def __init__(self):
        super().__init__('odom_line_controller')
        defaults = {
            'odom_topic': '/odom',
            'cmd_vel_topic': '/cmd_vel',
            'distance_m': 3.60,
            'max_linear_mps': 0.15,
            'max_angular_rps': 0.20,
            'linear_gain': 2.0,
            'angular_gain': 1.0,
            'waypoint_spacing_m': 4.00,
            'acceptance_radius_m': 0.20,
            'end_tolerance_m': 0.08,
            'max_cross_track_m': 1.00,
            'odom_timeout_s': 0.50,
            'max_run_time_s': 90.0,
            'control_rate_hz': 20.0,
            'enabled': False,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        def get(name):
            return self.get_parameter(name).value

        self.distance = float(get('distance_m'))
        self.max_linear = float(get('max_linear_mps'))
        self.max_angular = float(get('max_angular_rps'))
        self.linear_gain = float(get('linear_gain'))
        self.angular_gain = float(get('angular_gain'))
        self.waypoint_spacing = float(get('waypoint_spacing_m'))
        self.acceptance_radius = float(get('acceptance_radius_m'))
        self.end_tolerance = float(get('end_tolerance_m'))
        self.max_cross = float(get('max_cross_track_m'))
        self.odom_timeout = float(get('odom_timeout_s'))
        self.max_run_time = float(get('max_run_time_s'))
        rate = float(get('control_rate_hz'))
        positive = (
            self.distance, self.max_linear, self.max_angular,
            self.linear_gain, self.angular_gain, self.waypoint_spacing,
            self.acceptance_radius, self.end_tolerance, self.max_cross,
            self.odom_timeout, self.max_run_time, rate,
        )
        if not all(math.isfinite(value) and value > 0.0
                   for value in positive):
            raise ValueError('Controller parameters must be positive and finite')

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
        self.line = None
        self.line_yaw = None
        self.waypoints = []
        self.waypoint_index = 0
        self.previous_error = None
        self.last_warning_ns = 0
        self.get_logger().info(
            f'Ready for a {self.distance:.2f}m odom-relative line. '
            'Set enabled=true to capture the current pose and move.')

    def on_odom(self, msg):
        """Store the latest finite odometry pose and receipt time."""
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        pose = (
            float(position.x),
            float(position.y),
            yaw_from_quaternion(orientation),
        )
        if all(math.isfinite(value) for value in pose):
            self.pose = pose
            self.odom_time = self.get_clock().now()

    def stop(self):
        """Publish an all-zero velocity command."""
        self.publisher.publish(Twist())

    def warn_periodically(self, message):
        """Limit repeated warnings to one every two seconds."""
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self.last_warning_ns >= 2_000_000_000:
            self.get_logger().warn(message)
            self.last_warning_ns = now_ns

    def reset(self):
        """Forget the captured route so the next enable gets a new start."""
        self.finished = False
        self.start_time = None
        self.line = None
        self.line_yaw = None
        self.waypoints = []
        self.waypoint_index = 0
        self.previous_error = None

    def initialize_route(self):
        """Capture the current pose and construct the forward line."""
        x, y, yaw = self.pose
        self.line = relative_line(x, y, yaw, self.distance)
        self.line_yaw = yaw
        start_x, start_y, end_x, end_y = self.line
        self.waypoints = collinear_waypoints(
            start_x, start_y, end_x, end_y, self.waypoint_spacing)
        self.waypoint_index = 0
        self.start_time = self.get_clock().now()
        self.get_logger().info(
            f'Captured odom start=({start_x:.3f}, {start_y:.3f}), '
            f'yaw={yaw:.3f}rad; end=({end_x:.3f}, {end_y:.3f}), '
            f'waypoints={len(self.waypoints)}.')

    def on_timer(self):
        """Run one guarded odometry feedback iteration."""
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
            self.warn_periodically(
                f'Odometry is stale ({age:.2f}s); stopped.')
            return
        if self.finished:
            self.stop()
            return
        if self.line is None:
            self.initialize_route()

        x, y, yaw = self.pose
        start_x, start_y, end_x, end_y = self.line
        along, cross, _, length = line_errors(
            x, y, start_x, start_y, end_x, end_y)
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
                f'Line complete: along={along:.3f}m, '
                f'cross={cross:.3f}m; stopped.')
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
    """Run the odom-relative fixed-line controller."""
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    try:
        node = OdomLineController()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
