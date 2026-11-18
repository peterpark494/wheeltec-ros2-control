"""Paper equations (10), (15), and (18) for a fixed map line."""

import math


def wrap_angle(angle):
    """Wrap an angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def line_errors(x, y, start_x, start_y, end_x, end_y):
    """Return along-track position, signed cross-track error, yaw and length."""
    values = (x, y, start_x, start_y, end_x, end_y)
    if not all(math.isfinite(value) for value in values):
        raise ValueError('Pose and line coordinates must be finite')
    dx = end_x - start_x
    dy = end_y - start_y
    length = math.hypot(dx, dy)
    if length <= 0.0:
        raise ValueError('Line start and end must be different')
    ux = dx / length
    uy = dy / length
    rx = x - start_x
    ry = y - start_y
    along = ux * rx + uy * ry
    cross = -uy * rx + ux * ry
    return along, cross, math.atan2(dy, dx), length


def collinear_waypoints(start_x, start_y, end_x, end_y, spacing):
    """Generate equally spaced position references ending exactly at the end."""
    _, _, _, length = line_errors(
        start_x, start_y, start_x, start_y, end_x, end_y)
    if not math.isfinite(spacing) or spacing <= 0.0:
        raise ValueError('Waypoint spacing must be positive and finite')
    count = max(1, math.ceil(length / spacing))
    return [
        (start_x + (end_x - start_x) * i / count,
         start_y + (end_y - start_y) * i / count)
        for i in range(1, count + 1)
    ]


def relative_line(start_x, start_y, start_yaw, distance):
    """Return a line extending forward from an initial planar pose."""
    values = (start_x, start_y, start_yaw, distance)
    if not all(math.isfinite(value) for value in values):
        raise ValueError('Initial pose and distance must be finite')
    if distance <= 0.0:
        raise ValueError('Line distance must be positive')
    end_x = start_x + distance * math.cos(start_yaw)
    end_y = start_y + distance * math.sin(start_yaw)
    return start_x, start_y, end_x, end_y


def waypoint_complete(distance, acceptance_radius):
    """Paper radius-of-acceptance waypoint test."""
    values = (distance, acceptance_radius)
    if not all(math.isfinite(value) for value in values):
        raise ValueError('Waypoint progress values must be finite')
    if distance < 0.0 or acceptance_radius <= 0.0:
        raise ValueError('Distance must be nonnegative and radius positive')
    return distance <= acceptance_radius


def body_errors(x, y, yaw, target_x, target_y, target_yaw):
    """Equation (10): express waypoint error in the robot body frame."""
    values = (x, y, yaw, target_x, target_y, target_yaw)
    if not all(math.isfinite(value) for value in values):
        raise ValueError('Pose and target must be finite')
    dx = target_x - x
    dy = target_y - y
    e1 = math.cos(yaw) * dx + math.sin(yaw) * dy
    e2 = -math.sin(yaw) * dx + math.cos(yaw) * dy
    e3 = wrap_angle(target_yaw - yaw)
    return e1, e2, e3


def paper_velocity_command(previous_e1, previous_e3,
                           max_linear, max_angular,
                           linear_gain, angular_gain):
    """Equations (15) and (18), evaluated with the previous pose error."""
    values = (previous_e1, previous_e3, max_linear, max_angular,
              linear_gain, angular_gain)
    if not all(math.isfinite(value) for value in values):
        raise ValueError('Errors, velocity limits and gains must be finite')
    if min(max_linear, max_angular, linear_gain, angular_gain) <= 0.0:
        raise ValueError('Velocity limits and gains must be positive')
    desired_linear = max_linear * math.tanh(linear_gain * previous_e1)
    desired_angular = max_angular * math.tanh(angular_gain * previous_e3)
    return desired_linear, desired_angular


def backstepping_command(x, y, yaw, target_x, target_y, target_yaw,
                         max_linear, max_angular,
                         linear_gain, angular_gain):
    """
    Compute equations (15) and (18) body linear/angular velocities.

    The caller supplies the complete paper waypoint pose (x_r, y_r, theta_r).
    """
    positive = (max_linear, max_angular, linear_gain, angular_gain)
    if not all(math.isfinite(value) and value > 0.0 for value in positive):
        raise ValueError('Velocity limits and gains must be positive and finite')
    e1, e2, e3 = body_errors(
        x, y, yaw, target_x, target_y, target_yaw)
    desired_linear, desired_angular = paper_velocity_command(
        e1, e3, max_linear, max_angular, linear_gain, angular_gain)
    return desired_linear, desired_angular, e1, e2, e3
