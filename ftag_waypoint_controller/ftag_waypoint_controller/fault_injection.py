"""Velocity-layer approximation of a single front-left actuator fault."""

import math


def actuator_effectiveness(elapsed_s, minimum=0.7, decay_per_s=0.04):
    """Equation (65): return max(minimum, 1 - decay * elapsed)."""
    values = (elapsed_s, minimum, decay_per_s)
    if not all(math.isfinite(value) for value in values):
        raise ValueError('Fault parameters must be finite')
    if elapsed_s < 0.0:
        raise ValueError('Elapsed time must be nonnegative')
    if minimum <= 0.0 or minimum > 1.0:
        raise ValueError('Minimum effectiveness must be in (0, 1]')
    if decay_per_s < 0.0:
        raise ValueError('Effectiveness decay must be nonnegative')
    return max(minimum, 1.0 - decay_per_s * elapsed_s)


def front_left_fault_twist(linear, angular, effectiveness,
                           kinematic_span):
    """
    Project one weakened front-left wheel onto an equivalent body twist.

    The vendor 4WD firmware gives both left wheels v - span*w/2 and both
    right wheels v + span*w/2.  Only the front-left wheel is weakened.  The
    two wheels on each side are averaged before reconstructing the body
    velocity accepted by the existing ``cmd_vel`` interface.
    """
    values = (linear, angular, effectiveness, kinematic_span)
    if not all(math.isfinite(value) for value in values):
        raise ValueError('Velocity and fault values must be finite')
    if effectiveness <= 0.0 or effectiveness > 1.0:
        raise ValueError('Effectiveness must be in (0, 1]')
    if kinematic_span <= 0.0:
        raise ValueError('Kinematic span must be positive')

    left = linear - 0.5 * kinematic_span * angular
    right = linear + 0.5 * kinematic_span * angular
    effective_left = 0.5 * (effectiveness * left + left)
    effective_right = right
    output_linear = 0.5 * (effective_left + effective_right)
    output_angular = (effective_right - effective_left) / kinematic_span
    return output_linear, output_angular


def bounded_reconfigured_twist(linear, angular, effectiveness,
                               kinematic_span, wheel_speed_limit,
                               compensation_gain=1.0):
    """Solve the bounded left/right speed allocation under a FL fault.

    The vendor Twist interface gives the front and rear wheels on one side the
    same command.  A front-left efficiency ``lambda`` therefore produces the
    mean left-side efficiency ``(1 + lambda) / 2``.  In left/right wheel-speed
    coordinates the two allocation columns are orthogonal, so independently
    clipping the unconstrained solution is the exact bounded least-squares
    solution. ``compensation_gain`` models how much of that ideal allocation
    can be realised by a chassis that only accepts a body ``Twist`` command.

    Return the effective body twist, the allocated pre-fault body command,
    and a flag indicating whether either side reached its speed bound.
    """
    values = (linear, angular, effectiveness, kinematic_span,
              wheel_speed_limit, compensation_gain)
    if not all(math.isfinite(value) for value in values):
        raise ValueError('Velocity and allocation values must be finite')
    if effectiveness <= 0.0 or effectiveness > 1.0:
        raise ValueError('Effectiveness must be in (0, 1]')
    if kinematic_span <= 0.0 or wheel_speed_limit <= 0.0:
        raise ValueError('Kinematic span and wheel limit must be positive')
    if compensation_gain < 0.0 or compensation_gain > 1.0:
        raise ValueError('Compensation gain must be in [0, 1]')

    desired_left = linear - 0.5 * kinematic_span * angular
    desired_right = linear + 0.5 * kinematic_span * angular
    left_effectiveness = 0.5 * (1.0 + effectiveness)
    ideal_left = desired_left / left_effectiveness
    ideal_right = desired_right
    unconstrained_left = (
        desired_left + compensation_gain * (ideal_left - desired_left))
    unconstrained_right = (
        desired_right + compensation_gain * (ideal_right - desired_right))
    command_left = max(
        -wheel_speed_limit, min(wheel_speed_limit, unconstrained_left))
    command_right = max(
        -wheel_speed_limit, min(wheel_speed_limit, unconstrained_right))
    saturated = (
        not math.isclose(command_left, unconstrained_left,
                         rel_tol=0.0, abs_tol=1e-12) or
        not math.isclose(command_right, unconstrained_right,
                         rel_tol=0.0, abs_tol=1e-12)
    )

    allocated_linear = 0.5 * (command_left + command_right)
    allocated_angular = (command_right - command_left) / kinematic_span
    effective_left = left_effectiveness * command_left
    effective_right = command_right
    output_linear = 0.5 * (effective_left + effective_right)
    output_angular = (effective_right - effective_left) / kinematic_span
    return (output_linear, output_angular,
            allocated_linear, allocated_angular, saturated)
