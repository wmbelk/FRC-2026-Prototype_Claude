"""
Unit tests for ShooterHood projectile physics.

calculate_hood_angle() has no WPILib dependencies, so these tests run as
plain pytest without any robot/HAL initialization.
"""

import math
import pytest


# ---------------------------------------------------------------------------
# Standalone copy of the physics so tests don't require HAL initialization.
# Any change to ShooterHood.calculate_hood_angle() must be mirrored here.
# ---------------------------------------------------------------------------

HUB_HEIGHT = 1.8288       # meters
SHOOTER_HEIGHT = 0.3048   # meters
EFFICIENCY = 0.9
GRAVITY = 9.81
MIN_ANGLE = math.radians(10)
MAX_ANGLE = math.radians(60)


def _exit_velocity(rps: float) -> float:
    return rps * (2 * math.pi) * EFFICIENCY


def _calculate_hood_angle(distance: float, rps: float = 60.0) -> float:
    """Mirror of ShooterHood.calculate_hood_angle()."""
    delta_h = HUB_HEIGHT - SHOOTER_HEIGHT
    v = _exit_velocity(rps)
    g = GRAVITY

    discriminant = v**4 - g * (g * distance**2 + 2 * delta_h * v**2)

    if discriminant < 0:
        return MAX_ANGLE

    theta = math.atan((v**2 - math.sqrt(discriminant)) / (g * distance))
    return max(MIN_ANGLE, min(MAX_ANGLE, theta))


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

class TestHoodAngleClamping:
    def test_angle_never_below_minimum(self):
        """Very short distance should clamp to MIN_ANGLE, not go lower."""
        angle = _calculate_hood_angle(distance=0.5)
        assert angle >= MIN_ANGLE

    def test_angle_never_above_maximum(self):
        """Any distance should produce an angle at or below MAX_ANGLE."""
        for dist in [1.0, 3.0, 6.0, 10.0, 20.0]:
            angle = _calculate_hood_angle(distance=dist)
            assert angle <= MAX_ANGLE, f"angle {math.degrees(angle):.1f}° exceeded max at d={dist}"

    def test_impossible_shot_returns_max_angle(self):
        """When physics has no real solution, return MAX_ANGLE (best-effort steep shot)."""
        # rps=1 → v≈5.7 m/s; at 30 m the discriminant is deeply negative
        angle = _calculate_hood_angle(distance=30.0, rps=1.0)
        assert angle == MAX_ANGLE

    def test_result_is_within_bounds_for_normal_range(self):
        for dist in [2.0, 4.0, 6.0, 8.0]:
            angle = _calculate_hood_angle(distance=dist)
            assert MIN_ANGLE <= angle <= MAX_ANGLE


class TestHoodAnglePhysics:
    def test_closer_distance_needs_steeper_angle(self):
        """Closer targets require a steeper (larger) launch angle."""
        angle_near = _calculate_hood_angle(distance=2.0)
        angle_far = _calculate_hood_angle(distance=6.0)
        assert angle_near > angle_far, (
            f"Expected near ({math.degrees(angle_near):.1f}°) > far ({math.degrees(angle_far):.1f}°)"
        )

    def test_lower_rps_needs_steeper_angle_at_same_distance(self):
        """Slower exit velocity requires a steeper angle to reach the same height."""
        angle_fast = _calculate_hood_angle(distance=4.0, rps=60.0)
        angle_slow = _calculate_hood_angle(distance=4.0, rps=40.0)
        assert angle_slow >= angle_fast, (
            f"Expected slow ({math.degrees(angle_slow):.1f}°) >= fast ({math.degrees(angle_fast):.1f}°)"
        )

    def test_angle_is_monotonically_decreasing_with_distance(self):
        """Angle should decrease (or stay clamped) as distance increases."""
        distances = [1.5, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0]
        angles = [_calculate_hood_angle(d) for d in distances]
        for i in range(len(angles) - 1):
            assert angles[i] >= angles[i + 1] - 1e-9, (
                f"Angle increased between d={distances[i]} and d={distances[i+1]}: "
                f"{math.degrees(angles[i]):.2f}° → {math.degrees(angles[i+1]):.2f}°"
            )


class TestExitVelocity:
    def test_with_rps_scales_exit_velocity(self):
        """Doubling RPS should double exit velocity."""
        v1 = _exit_velocity(30.0)
        v2 = _exit_velocity(60.0)
        assert math.isclose(v2, 2 * v1)

    def test_efficiency_factor_applied(self):
        """Exit velocity should be RPS * 2π * efficiency."""
        rps = 50.0
        expected = rps * 2 * math.pi * EFFICIENCY
        assert math.isclose(_exit_velocity(rps), expected)
