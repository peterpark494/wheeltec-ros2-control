"""Approximate a front-left motor effectiveness loss at velocity level."""

import math

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Float64

from .fault_injection import (
    actuator_effectiveness,
    bounded_reconfigured_twist,
)


class FrontLeftFaultInjector(Node):
    """Relay nominal velocity through an equivalent front-left wheel fault."""

    def __init__(self):
        super().__init__('front_left_fault_injector')
        defaults = {
            'input_cmd_vel_topic': '/cmd_vel_nominal',
            'output_cmd_vel_topic': '/cmd_vel',
            'allocated_cmd_vel_topic': '/cmd_vel_allocated',
            'effectiveness_topic': '/fault/effectiveness',
            'fault_enabled': False,
            'minimum_effectiveness': 0.70,
            'decay_per_s': 0.04,
            'kinematic_span_m': 0.360,
            'max_wheel_linear_mps': 0.20,
            'compensation_gain': 0.80,
            'command_timeout_s': 0.50,
            'max_output_linear_mps': 0.20,
            'max_output_angular_rps': 0.30,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        def get(name):
            return self.get_parameter(name).value

        self.minimum = float(get('minimum_effectiveness'))
        self.decay = float(get('decay_per_s'))
        self.span = float(get('kinematic_span_m'))
        self.wheel_limit = float(get('max_wheel_linear_mps'))
        self.compensation_gain = float(get('compensation_gain'))
        self.timeout = float(get('command_timeout_s'))
        self.max_linear = float(get('max_output_linear_mps'))
        self.max_angular = float(get('max_output_angular_rps'))
        actuator_effectiveness(0.0, self.minimum, self.decay)
        positive = (
            self.span, self.wheel_limit, self.timeout,
            self.max_linear, self.max_angular,
        )
        if not all(math.isfinite(value) and value > 0.0
                   for value in positive):
            raise ValueError('Injector limits must be positive and finite')
        if (not math.isfinite(self.compensation_gain) or
                self.compensation_gain < 0.0 or
                self.compensation_gain > 1.0):
            raise ValueError('Compensation gain must be in [0, 1]')

        self.publisher = self.create_publisher(
            Twist, str(get('output_cmd_vel_topic')), 10)
        self.allocated_publisher = self.create_publisher(
            Twist, str(get('allocated_cmd_vel_topic')), 10)
        self.effectiveness_publisher = self.create_publisher(
            Float64, str(get('effectiveness_topic')), 10)
        self.subscription = self.create_subscription(
            Twist, str(get('input_cmd_vel_topic')), self.on_command, 10)
        self.timer = self.create_timer(0.05, self.on_timer)
        self.last_command_time = None
        self.fault_start_time = None
        self.was_enabled = False
        self.timed_out = False
        self.last_log_second = -1
        self.last_saturation_warning_ns = 0
        self.publish_effectiveness(1.0)
        self.get_logger().info(
            'Ready. Enabling the fault activates the bounded left/right '
            'reconfiguration; '
            f'compensation_gain={self.compensation_gain:.2f}. The fault '
            'clock starts on the first nonzero command.')

    @staticmethod
    def clamp(value, limit):
        """Symmetrically clamp a scalar command."""
        return max(-limit, min(limit, value))

    def publish_effectiveness(self, value):
        """Publish the current actuator effectiveness factor."""
        message = Float64()
        message.data = float(value)
        self.effectiveness_publisher.publish(message)

    def stop(self):
        """Publish an all-zero velocity command."""
        self.publisher.publish(Twist())

    def reset_fault_clock(self):
        """Reset the experiment clock for the next fault run."""
        self.fault_start_time = None
        self.last_log_second = -1

    def sync_enabled_state(self):
        """Apply parameter transitions even when no command is arriving."""
        enabled = bool(self.get_parameter('fault_enabled').value)
        if enabled == self.was_enabled:
            return enabled
        self.was_enabled = enabled
        if enabled:
            self.get_logger().info(
                'Fault armed with bounded left/right reconfiguration; '
                'waiting for the first nonzero command.')
        else:
            self.reset_fault_clock()
            self.publish_effectiveness(1.0)
            self.get_logger().info('Fault disabled; clock reset.')
        return enabled

    def on_command(self, msg):
        """Apply the configured approximation to one nominal Twist."""
        now = self.get_clock().now()
        self.last_command_time = now
        self.timed_out = False
        enabled = self.sync_enabled_state()

        if not enabled:
            self.publish_effectiveness(1.0)
            self.publisher.publish(msg)
            return

        moving = (abs(float(msg.linear.x)) > 1e-6 or
                  abs(float(msg.angular.z)) > 1e-6)
        if self.fault_start_time is None and moving:
            self.fault_start_time = now
            self.get_logger().info(
                'Front-left fault started: lambda=max('
                f'{self.minimum:.2f}, 1-{self.decay:.3f}t).')

        if self.fault_start_time is None:
            effectiveness = 1.0
        else:
            elapsed = (now - self.fault_start_time).nanoseconds / 1e9
            effectiveness = actuator_effectiveness(
                elapsed, self.minimum, self.decay)
            whole_second = int(elapsed)
            if whole_second != self.last_log_second:
                self.get_logger().info(
                    f'Fault t={elapsed:.1f}s, lambda={effectiveness:.3f}.')
                self.last_log_second = whole_second

        (output_linear, output_angular,
         allocated_linear, allocated_angular,
         saturated) = bounded_reconfigured_twist(
            float(msg.linear.x), float(msg.angular.z),
            effectiveness, self.span, self.wheel_limit,
            self.compensation_gain)
        allocated = Twist()
        allocated.linear.x = float(allocated_linear)
        allocated.angular.z = float(allocated_angular)
        self.allocated_publisher.publish(allocated)
        if saturated:
            now_ns = now.nanoseconds
            if now_ns - self.last_saturation_warning_ns >= 2_000_000_000:
                self.get_logger().warn(
                    'Bounded side allocation is saturated; exact fault '
                    'compensation is not possible at this command.')
                self.last_saturation_warning_ns = now_ns
        command = Twist()
        command.linear.x = self.clamp(output_linear, self.max_linear)
        command.angular.z = self.clamp(output_angular, self.max_angular)
        self.publish_effectiveness(effectiveness)
        self.publisher.publish(command)

    def on_timer(self):
        """Stop the chassis if the nominal command stream disappears."""
        self.sync_enabled_state()
        if self.last_command_time is None:
            return
        age = ((self.get_clock().now() - self.last_command_time).nanoseconds /
               1e9)
        if age > self.timeout and not self.timed_out:
            self.stop()
            self.timed_out = True
            self.get_logger().warn(
                f'Nominal command timeout ({age:.2f}s); stopped.')

    def destroy_node(self):
        if rclpy.ok():
            self.stop()
        return super().destroy_node()


def main(args=None):
    """Run the velocity-level front-left fault injector."""
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    try:
        node = FrontLeftFaultInjector()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
