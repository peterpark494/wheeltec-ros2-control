"""Tests for the odometry-relative quadratic reference."""

import math

from ftag_waypoint_controller.curve_guidance import (
    quadratic_cross_track,
    quadratic_curve_waypoints,
    start_frame_position,
)


def test_quadratic_curve_reaches_requested_endpoint():
    points = quadratic_curve_waypoints(0.0, 0.0, 0.0, 3.2, 0.05, 0.1)
    x, y, yaw, local_x = points[-1]
    assert math.isclose(x, 3.2)
    assert math.isclose(y, 0.05 * 3.2 ** 2)
    assert math.isclose(yaw, math.atan(0.1 * 3.2))
    assert math.isclose(local_x, 3.2)


def test_curve_is_rotated_from_the_captured_start_pose():
    points = quadratic_curve_waypoints(
        1.0, 2.0, math.pi / 2.0, 1.0, 0.05, 1.0)
    x, y, yaw, _ = points[-1]
    assert math.isclose(x, 0.95, abs_tol=1e-12)
    assert math.isclose(y, 3.0, abs_tol=1e-12)
    assert math.isclose(yaw, math.pi / 2.0 + math.atan(0.1))


def test_start_frame_position_inverts_pose_rotation():
    local_x, local_y = start_frame_position(
        0.95, 3.0, 1.0, 2.0, math.pi / 2.0)
    assert math.isclose(local_x, 1.0, abs_tol=1e-12)
    assert math.isclose(local_y, 0.05, abs_tol=1e-12)


def test_quadratic_cross_track_is_signed():
    assert math.isclose(quadratic_cross_track(2.0, 0.3, 0.05), 0.1)
    assert math.isclose(quadratic_cross_track(2.0, 0.1, 0.05), -0.1)
