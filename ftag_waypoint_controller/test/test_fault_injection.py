"""Tests for the velocity-level front-left fault approximation."""

import math

from ftag_waypoint_controller.fault_injection import (
    actuator_effectiveness,
    bounded_reconfigured_twist,
    front_left_fault_twist,
)


def test_equation_65_reaches_and_holds_minimum():
    assert actuator_effectiveness(0.0) == 1.0
    assert actuator_effectiveness(5.0) == 0.8
    assert actuator_effectiveness(7.5) == 0.7
    assert actuator_effectiveness(20.0) == 0.7


def test_healthy_motor_preserves_body_command():
    linear, angular = front_left_fault_twist(0.08, -0.1, 1.0, 0.36)
    assert math.isclose(linear, 0.08)
    assert math.isclose(angular, -0.1)


def test_straight_front_left_fault_creates_expected_yaw():
    linear, angular = front_left_fault_twist(0.08, 0.0, 0.7, 0.36)
    assert math.isclose(linear, 0.925 * 0.08)
    assert math.isclose(angular, 0.15 * 0.08 / 0.36)


def test_bounded_reconfiguration_recovers_nominal_straight_twist():
    output_v, output_w, allocated_v, allocated_w, saturated = (
        bounded_reconfigured_twist(0.15, 0.0, 0.70, 0.36, 0.20))
    assert math.isclose(output_v, 0.15, abs_tol=1e-12)
    assert math.isclose(output_w, 0.0, abs_tol=1e-12)
    assert allocated_v > 0.15
    assert allocated_w < 0.0
    assert not saturated


def test_bounded_reconfiguration_reports_saturation():
    output_v, output_w, _, _, saturated = bounded_reconfigured_twist(
        0.30, 0.0, 0.70, 0.36, 0.20)
    assert saturated
    assert output_v < 0.30
    assert output_w != 0.0


def test_partial_reconfiguration_retains_residual_fault():
    output_v, output_w, _, _, saturated = bounded_reconfigured_twist(
        0.15, 0.0, 0.70, 0.36, 0.20, compensation_gain=0.90)
    assert output_v < 0.15
    assert output_w > 0.0
    assert not saturated
