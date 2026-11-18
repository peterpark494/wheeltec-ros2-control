"""FT-AG paper equations (10), (15), (18): kinematic waypoint outer loop."""

import math

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y),
                      1 - 2 * (q.y * q.y + q.z * q.z))


def guidance(x, y, yaw, gx, gy, vmax, wmax, kv, kw, heading_gate):
    """Calculate body velocity toward a position waypoint."""
    dx, dy = gx - x, gy - y
    distance = math.hypot(dx, dy)
    e1 = math.cos(yaw) * dx + math.sin(yaw) * dy
    e3 = wrap_angle(math.atan2(dy, dx) - yaw)
    v = vmax * math.tanh(kv * e1)
    w = wmax * math.tanh(kw * e3)
    # Explicit real-robot adaptation: turn first if the target is far aside.
    if abs(e3) > heading_gate:
        v = 0.0
    return v, w, distance


class FtagWaypointController(Node):
    """Guide the robot through position waypoints using odometry."""

    def __init__(self):
        super().__init__('ftag_waypoint_controller')
        defaults = {
            'odom_topic': '/odom', 'cmd_vel_topic': '/cmd_vel',
            'control_rate_hz': 20.0, 'max_linear_mps': 0.10,
            'max_angular_rps': 0.35, 'linear_gain': 2.0,
            'angular_gain': 1.5, 'heading_gate_rad': 0.8,
            'acceptance_radius_m': 0.12, 'odom_timeout_s': 0.5,
            'relative_to_start': True, 'enabled': False,
            'waypoints_x': [0.5], 'waypoints_y': [0.0],
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        def get(name):
            return self.get_parameter(name).value

        self.xs, self.ys = list(get('waypoints_x')), list(get('waypoints_y'))
        if not self.xs or len(self.xs) != len(self.ys):
            raise ValueError('waypoints_x and waypoints_y need equal nonzero length')
        if not all(math.isfinite(float(n)) for n in self.xs + self.ys):
            raise ValueError('Waypoints must be finite numbers')
        self.rate = float(get('control_rate_hz'))
        self.vmax = float(get('max_linear_mps'))
        self.wmax = float(get('max_angular_rps'))
        self.kv = float(get('linear_gain'))
        self.kw = float(get('angular_gain'))
        self.gate = float(get('heading_gate_rad'))
        self.radius = float(get('acceptance_radius_m'))
        self.timeout = float(get('odom_timeout_s'))
        if not all(math.isfinite(n) and n > 0 for n in
                   [self.rate, self.vmax, self.wmax, self.kv, self.kw,
                    self.gate, self.radius, self.timeout]):
            raise ValueError('Rate, limits, gains, gate, radius, timeout must be positive')
        self.relative = bool(get('relative_to_start'))
        self.pose = None
        self.odom_time = None
        self.goals = None
        self.index = 0
        self.complete = False
        self.was_enabled = False
        self.last_warning_ns = 0
        self.pub = self.create_publisher(Twist, str(get('cmd_vel_topic')), 10)
        self.sub = self.create_subscription(
            Odometry, str(get('odom_topic')), self.on_odom, 10)
        self.timer = self.create_timer(1.0 / self.rate, self.on_timer)
        self.get_logger().info('Ready. Set enabled=true to move.')

    def on_odom(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        pose = (float(p.x), float(p.y), yaw_from_quaternion(q))
        if all(math.isfinite(n) for n in pose):
            self.pose = pose
            self.odom_time = self.get_clock().now()

    def stop(self):
        self.pub.publish(Twist())

    def on_timer(self):
        enabled = bool(self.get_parameter('enabled').value)
        if not enabled:
            if self.was_enabled:
                self.get_logger().info('Disabled; stopped.')
                self.stop()
                self.goals = None
                self.index = 0
                self.complete = False
            self.was_enabled = False
            return
        self.was_enabled = True
        if self.pose is None or self.odom_time is None:
            self.stop()
            return
        age = (self.get_clock().now() - self.odom_time).nanoseconds / 1e9
        if age < 0 or age > self.timeout:
            self.stop()
            now_ns = self.get_clock().now().nanoseconds
            if now_ns - self.last_warning_ns > 2_000_000_000:
                self.get_logger().warn(f'Odometry stale ({age:.2f}s); stopped.')
                self.last_warning_ns = now_ns
            return
        if self.complete:
            self.stop()
            return
        x, y, yaw = self.pose
        if self.goals is None:
            if self.relative:
                c, s = math.cos(yaw), math.sin(yaw)
                self.goals = [(x + c*dx - s*dy, y + s*dx + c*dy)
                              for dx, dy in zip(self.xs, self.ys)]
            else:
                self.goals = list(zip(self.xs, self.ys))
            self.get_logger().info(f'Goals in odom frame: {self.goals}')
        while self.index < len(self.goals):
            gx, gy = self.goals[self.index]
            if math.hypot(gx - x, gy - y) >= self.radius:
                break
            self.index += 1
            self.get_logger().info(f'Reached {self.index}/{len(self.goals)}')
        if self.index == len(self.goals):
            self.complete = True
            self.stop()
            self.get_logger().info('All waypoints reached; stopped.')
            return
        gx, gy = self.goals[self.index]
        v, w, _ = guidance(x, y, yaw, gx, gy, self.vmax, self.wmax,
                           self.kv, self.kw, self.gate)
        cmd = Twist()
        cmd.linear.x, cmd.angular.z = float(v), float(w)
        self.pub.publish(cmd)

    def destroy_node(self):
        if self.was_enabled and rclpy.ok():
            self.stop()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    try:
        node = FtagWaypointController()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
