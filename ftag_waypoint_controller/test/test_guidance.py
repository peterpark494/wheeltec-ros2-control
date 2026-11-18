"""Checks for the waypoint control law used on the real robot."""

import math

from ftag_waypoint_controller.controller_node import guidance, wrap_angle


def test_straight_ahead_is_forward_only():
    v, w, distance = guidance(0, 0, 0, 0.5, 0, 0.1, 0.35, 2, 1.5, 0.8)
    assert 0 < v < 0.1
    assert w == 0
    assert distance == 0.5


def test_side_waypoint_turns_before_driving():
    v, w, _ = guidance(0, 0, 0, 0, 0.5, 0.1, 0.35, 2, 1.5, 0.8)
    assert v == 0
    assert 0 < w < 0.35


def test_heading_wraps_across_pi():
    assert abs(wrap_angle(2 * math.pi + 0.1) - 0.1) < 1e-12
