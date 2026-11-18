"""Unit tests for the paper kinematic outer loop and line references."""

import math

from ftag_waypoint_controller.line_guidance import (
    backstepping_command,
    body_errors,
    collinear_waypoints,
    line_errors,
    paper_velocity_command,
    relative_line,
    waypoint_complete,
)


def test_line_errors_are_signed():
    along, cross, yaw, length = line_errors(1.0, 1.0, 0.0, 0.0, 2.0, 0.0)
    assert (along, cross, yaw, length) == (1.0, 1.0, 0.0, 2.0)
    assert line_errors(1.0, -1.0, 0.0, 0.0, 2.0, 0.0)[1] == -1.0


def test_collinear_waypoints_include_exact_end():
    points = collinear_waypoints(0.0, 0.0, 1.0, 0.0, 0.3)
    assert len(points) == 4
    assert points[-1] == (1.0, 0.0)
    assert all(y == 0.0 for _, y in points)


def test_relative_line_uses_initial_heading():
    start_x, start_y, end_x, end_y = relative_line(
        1.0, 2.0, math.pi / 2.0, 1.8)
    assert (start_x, start_y) == (1.0, 2.0)
    assert math.isclose(end_x, 1.0, abs_tol=1e-12)
    assert math.isclose(end_y, 3.8, abs_tol=1e-12)


def test_waypoint_advances_only_inside_paper_acceptance_radius():
    assert waypoint_complete(0.19, 0.20)
    assert not waypoint_complete(0.25, 0.20)


def test_equation_10_body_transform():
    e1, e2, e3 = body_errors(
        0.0, 0.0, math.pi / 2.0, 0.0, 1.0, math.pi / 2.0)
    assert math.isclose(e1, 1.0, abs_tol=1e-12)
    assert math.isclose(e2, 0.0, abs_tol=1e-12)
    assert math.isclose(e3, 0.0, abs_tol=1e-12)


def test_equations_15_and_18():
    v, w, e1, e2, e3 = backstepping_command(
        0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.1, 0.35, 2.0, 1.5)
    assert math.isclose(v, 0.1 * math.tanh(2.0))
    assert w == 0.0
    assert (e1, e2, e3) == (1.0, 0.0, 0.0)


def test_heading_error_generates_bounded_turn():
    _, w, _, _, e3 = backstepping_command(
        0.0, 0.0, 0.0, 0.0, 1.0, math.pi / 2.0,
        0.1, 0.35, 2.0, 1.5)
    assert e3 == math.pi / 2.0
    assert 0.0 < w < 0.35


def test_fixed_reference_yaw_does_not_correct_pure_lateral_error():
    _, w, _, e2, e3 = backstepping_command(
        0.0, 1.0, 0.0, 1.0, 0.0, 0.0,
        0.1, 0.35, 2.0, 1.5)
    assert e2 == -1.0
    assert e3 == 0.0
    assert w == 0.0


def test_paper_velocity_uses_supplied_previous_error():
    v, w = paper_velocity_command(0.5, -0.2, 0.1, 0.35, 2.0, 1.5)
    assert math.isclose(v, 0.1 * math.tanh(1.0))
    assert math.isclose(w, 0.35 * math.tanh(-0.3))
