"""Reference generation helpers for an odometry-relative quadratic curve."""

import math

from .line_guidance import wrap_angle


def quadratic_curve_waypoints(start_x, start_y, start_yaw, x_length,
                              coefficient, spacing):
    """Generate pose waypoints for local ``y = coefficient * x**2``."""
    values = (start_x, start_y, start_yaw, x_length,
              coefficient, spacing)
    if not all(math.isfinite(value) for value in values):
        raise ValueError('Curve parameters must be finite')
    if x_length <= 0.0 or spacing <= 0.0 or coefficient < 0.0:
        raise ValueError(
            'Curve length and spacing must be positive; coefficient '
            'must be nonnegative')

    count = max(1, math.ceil(x_length / spacing))
    cos_yaw = math.cos(start_yaw)
    sin_yaw = math.sin(start_yaw)
    waypoints = []
    for index in range(1, count + 1):
        local_x = x_length * index / count
        local_y = coefficient * local_x * local_x
        world_x = start_x + cos_yaw * local_x - sin_yaw * local_y
        world_y = start_y + sin_yaw * local_x + cos_yaw * local_y
        tangent_yaw = wrap_angle(
            start_yaw + math.atan(2.0 * coefficient * local_x))
        waypoints.append((world_x, world_y, tangent_yaw, local_x))
    return waypoints


def start_frame_position(x, y, start_x, start_y, start_yaw):
    """Express an odometry position in the captured start-pose frame."""
    values = (x, y, start_x, start_y, start_yaw)
    if not all(math.isfinite(value) for value in values):
        raise ValueError('Pose values must be finite')
    dx = x - start_x
    dy = y - start_y
    local_x = math.cos(start_yaw) * dx + math.sin(start_yaw) * dy
    local_y = -math.sin(start_yaw) * dx + math.cos(start_yaw) * dy
    return local_x, local_y


def quadratic_cross_track(local_x, local_y, coefficient):
    """Return vertical displacement from the quadratic reference."""
    values = (local_x, local_y, coefficient)
    if not all(math.isfinite(value) for value in values):
        raise ValueError('Curve error values must be finite')
    return local_y - coefficient * local_x * local_x
