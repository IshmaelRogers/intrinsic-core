# Copyright 2026 Intrinsic Innovation LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Tests for the vehicle StateSpace Python mirror."""

import dataclasses
import math
import random
import unittest

from intrinsic.embodiment import frame_policy
from intrinsic.motion_planning.vehicle import vehicle_state_space as vss

_TOL = 1e-9
_HALF_SQRT2 = math.sqrt(2.0) / 2.0
_OK = vss.StateSpaceError.OK


def _state(
    position=(0.0, 0.0, 0.0),
    orientation=(0.0, 0.0, 0.0, 1.0),
    twist=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
):
  return vss.VehiclePlanningState(
      position=position, orientation=orientation, twist=twist
  )


def _yaw(angle):
  return (0.0, 0.0, math.sin(angle / 2), math.cos(angle / 2))


def _negated(q):
  return tuple(-c for c in q)


def _fixture_a():
  return _state()


def _fixture_b():
  return _state(
      position=(2.0, -4.0, 6.0),
      orientation=(0.0, 0.0, _HALF_SQRT2, _HALF_SQRT2),
      twist=(1.0, 2.0, 3.0, 0.5, -1.0, 0.25),
  )


class StateSpaceTestCase(unittest.TestCase):

  def assert_state_near(self, actual, expected, tol=_TOL):
    for name in ("position", "orientation", "twist"):
      got = getattr(actual, name)
      want = getattr(expected, name)
      self.assertEqual(len(got), len(want), name)
      for g, w in zip(got, want):
        self.assertAlmostEqual(g, w, delta=tol, msg=name)


class InterpolateTest(StateSpaceTestCase):

  def test_endpoints_return_inputs_exactly(self):
    a, b = _fixture_a(), _fixture_b()
    at_a = vss.interpolate(a, b, 0.0)
    at_b = vss.interpolate(a, b, 1.0)
    self.assertIs(at_a.error, _OK)
    self.assertIs(at_b.error, _OK)
    self.assertEqual(at_a.state, a)
    self.assertEqual(at_b.state, b)

  def test_endpoint_returns_b_when_shortest_path_flips_sign(self):
    a = _fixture_a()
    b = _state(position=(1.0, 1.0, 1.0), orientation=_negated(_yaw(1.0)))
    result = vss.interpolate(a, b, 1.0)
    self.assertIs(result.error, _OK)
    self.assertEqual(result.state, b)

  def test_midpoint_golden(self):
    result = vss.interpolate(_fixture_a(), _fixture_b(), 0.5)
    self.assertIs(result.error, _OK)
    expected = _state(
        position=(1.0, -2.0, 3.0),
        orientation=(0.0, 0.0, 0.38268343236508978, 0.92387953251128674),
        twist=(0.5, 1.0, 1.5, 0.25, -0.5, 0.125),
    )
    self.assert_state_near(result.state, expected)

  def test_midpoint_golden_with_non_identity_start(self):
    a = _state(
        position=(1.0, 2.0, 3.0),
        orientation=_yaw(math.pi / 6),
        twist=(-1.0, 0.0, 1.0, 0.0, 0.0, 2.0),
    )
    b = _state(
        position=(3.0, -2.0, 7.0),
        orientation=_yaw(math.pi / 2),
        twist=(1.0, 0.0, -1.0, 0.0, 0.0, 4.0),
    )
    result = vss.interpolate(a, b, 0.5)
    self.assertIs(result.error, _OK)
    expected = _state(
        position=(2.0, 0.0, 5.0),
        orientation=(0.0, 0.0, 0.5, 0.86602540378443865),
        twist=(0.0, 0.0, 0.0, 0.0, 0.0, 3.0),
    )
    self.assert_state_near(result.state, expected)

  def test_quarter_point_uses_slerp_not_nlerp(self):
    a = _state()
    b = _state(orientation=_yaw(math.pi))
    result = vss.interpolate(a, b, 0.25)
    self.assertIs(result.error, _OK)
    expected = _yaw(math.pi / 4)
    self.assertAlmostEqual(result.state.orientation[2], expected[2], delta=_TOL)
    self.assertAlmostEqual(result.state.orientation[3], expected[3], delta=_TOL)

  def test_shortest_path_takes_acute_arc_for_negative_dot(self):
    total = 2 * math.pi / 3
    a = _state()
    b = _state(orientation=_negated(_yaw(total)))
    self.assertLess(a.orientation[3] * b.orientation[3], 0.0)

    previous_from_a = -1.0
    previous_to_b = math.inf
    for i in range(21):
      u = i / 20.0
      result = vss.interpolate(a, b, u)
      self.assertIs(result.error, _OK)
      at_u = _state(orientation=result.state.orientation)
      from_a = vss.distance(a, at_u)
      to_b = vss.distance(at_u, b)
      self.assertAlmostEqual(from_a, u * total, delta=_TOL)
      self.assertGreater(from_a, previous_from_a)
      self.assertLess(to_b, previous_to_b)
      previous_from_a, previous_to_b = from_a, to_b

    mid = vss.interpolate(a, b, 0.5)
    self.assertAlmostEqual(mid.state.orientation[2], 0.5, delta=_TOL)
    self.assertAlmostEqual(
        mid.state.orientation[3], 0.86602540378443865, delta=_TOL
    )

  def test_sign_flipped_endpoint_gives_same_rotations(self):
    a = _state(orientation=_yaw(0.3))
    b = _state(orientation=_yaw(0.3 + math.pi / 2))
    b_flipped = _state(orientation=_negated(b.orientation))
    for u in (0.1, 0.25, 0.5, 0.9):
      plain = vss.interpolate(a, b, u)
      flipped = vss.interpolate(a, b_flipped, u)
      self.assert_state_near(flipped.state, plain.state)

  def test_equal_orientations_are_stable(self):
    q = _yaw(0.7)
    result = vss.interpolate(
        _state(orientation=q),
        _state(position=(1.0, 0.0, 0.0), orientation=q),
        0.3,
    )
    self.assertIs(result.error, _OK)
    self.assertAlmostEqual(result.state.orientation[2], q[2], delta=_TOL)
    self.assertAlmostEqual(result.state.orientation[3], q[3], delta=_TOL)
    self.assertAlmostEqual(result.state.position[0], 0.3, delta=_TOL)

  def test_antipodal_pair_folds_to_first_orientation(self):
    q = _yaw(1.1)
    a = _state(orientation=q)
    b = _state(orientation=_negated(q))
    for u in (0.1, 0.5, 0.9):
      result = vss.interpolate(a, b, u)
      self.assertIs(result.error, _OK)
      for got, want in zip(result.state.orientation, q):
        self.assertAlmostEqual(got, want, delta=_TOL)

  def test_near_pi_branch_matches_eigenmath(self):
    # 180 degrees about x: dot(a, b) is 0, which eigenmath does not flip, so
    # the tie breaks toward +90 degrees about x. Negating b picks -90.
    a = _state()
    b = _state(orientation=(1.0, 0.0, 0.0, 0.0))
    b_negated = _state(orientation=(-1.0, 0.0, 0.0, 0.0))

    positive = vss.interpolate(a, b, 0.5)
    self.assertIs(positive.error, _OK)
    for got, want in zip(
        positive.state.orientation, (_HALF_SQRT2, 0.0, 0.0, _HALF_SQRT2)
    ):
      self.assertAlmostEqual(got, want, delta=_TOL)

    negative = vss.interpolate(a, b_negated, 0.5)
    self.assertIs(negative.error, _OK)
    for got, want in zip(
        negative.state.orientation, (-_HALF_SQRT2, 0.0, 0.0, _HALF_SQRT2)
    ):
      self.assertAlmostEqual(got, want, delta=_TOL)

  def test_mix_parameter_outside_unit_interval_is_rejected(self):
    a, b = _fixture_a(), _fixture_b()
    for u in (-1e-12, -0.5, 1.0 + 1e-12, 2.0, math.nan, math.inf, -math.inf):
      self.assertIs(
          vss.interpolate(a, b, u).error, vss.StateSpaceError.MIX_PARAMETER, u
      )

  def test_non_finite_inputs_are_rejected(self):
    good = _fixture_a()
    for bad in (math.nan, math.inf, -math.inf):
      bad_states = (
          _state(position=(0.0, bad, 0.0)),
          _state(orientation=(bad, 0.0, 0.0, 1.0)),
          _state(twist=(0.0, 0.0, 0.0, 0.0, 0.0, bad)),
      )
      for state in bad_states:
        self.assertIs(
            vss.interpolate(state, good, 0.5).error,
            vss.StateSpaceError.NON_FINITE,
        )
        self.assertIs(
            vss.interpolate(good, state, 0.5).error,
            vss.StateSpaceError.NON_FINITE,
        )

  def test_non_unit_orientation_is_rejected_not_renormalized(self):
    good = _fixture_a()
    for orientation in (
        (0.0, 0.0, 0.0, 2.0),
        (0.0, 0.0, 0.0, 0.0),
        (0.0, 0.0, 0.0, 1.0 + 1e-8),
    ):
      bad = _state(orientation=orientation)
      for u in (0.0, 0.5, 1.0):
        self.assertIs(
            vss.interpolate(bad, good, u).error, vss.StateSpaceError.QUATERNION
        )
        self.assertIs(
            vss.interpolate(good, bad, u).error, vss.StateSpaceError.QUATERNION
        )

  def test_check_order_is_mix_then_finite_then_quaternion(self):
    bad = _state(position=(math.nan, 0.0, 0.0), orientation=(0, 0, 0, 2.0))
    b = _fixture_b()
    self.assertIs(
        vss.interpolate(bad, b, 2.0).error, vss.StateSpaceError.MIX_PARAMETER
    )
    self.assertIs(
        vss.interpolate(bad, b, 0.5).error, vss.StateSpaceError.NON_FINITE
    )
    bad = _state(orientation=(0, 0, 0, 2.0))
    self.assertIs(
        vss.interpolate(bad, b, 0.5).error, vss.StateSpaceError.QUATERNION
    )

  def test_accepts_inputs_within_norm_tolerance(self):
    a = _state(orientation=(0.0, 0.0, 0.0, 1.0 + 5e-10))
    result = vss.interpolate(a, _fixture_b(), 0.5)
    self.assertIs(result.error, _OK)
    self.assertAlmostEqual(
        frame_policy.quaternion_norm(result.state.orientation), 1.0, delta=1e-15
    )


class DistanceTest(StateSpaceTestCase):

  def test_zero_for_identical_states(self):
    self.assertEqual(vss.distance(_fixture_b(), _fixture_b()), 0.0)

  def test_same_rotation_with_opposite_sign_is_zero_angle(self):
    a = _state(orientation=_yaw(0.8))
    b = _state(orientation=_negated(_yaw(0.8)))
    self.assertAlmostEqual(vss.distance(a, b), 0.0, delta=_TOL)

  def test_combines_all_terms_with_unit_weights(self):
    a = _state()
    b = _state(
        position=(2.0, 3.0, 6.0),
        orientation=_yaw(math.pi / 2),
        twist=(1.0, 2.0, 2.0, 0.0, 0.0, 4.0),
    )
    expected = math.sqrt(49.0 + (math.pi / 2) ** 2 + 9.0 + 16.0)
    self.assertAlmostEqual(vss.distance(a, b), expected, delta=_TOL)

  def test_angle_is_geodesic_and_at_most_pi(self):
    a = _state()
    self.assertAlmostEqual(
        vss.distance(a, _state(orientation=_yaw(math.pi))), math.pi, delta=_TOL
    )
    self.assertAlmostEqual(
        vss.distance(a, _state(orientation=_yaw(1.5 * math.pi))),
        math.pi / 2,
        delta=_TOL,
    )
    self.assertAlmostEqual(
        vss.distance(a, _state(orientation=_yaw(1e-7))), 1e-7, delta=1e-15
    )

  def test_infinite_for_invalid_inputs(self):
    good = _fixture_a()
    bad_states = (
        _state(position=(math.nan, 0.0, 0.0)),
        _state(twist=(0.0, 0.0, math.inf, 0.0, 0.0, 0.0)),
        _state(orientation=(0.0, 0.0, 0.0, 2.0)),
    )
    for bad in bad_states:
      self.assertEqual(vss.distance(good, bad), math.inf)
      self.assertEqual(vss.distance(bad, good), math.inf)
      self.assertEqual(vss.distance(bad, bad), math.inf)


def _engaged_bounds():
  return vss.VehicleStateBounds(
      position_limits_present=True,
      position_min=(-10.0, -10.0, -20.0),
      position_max=(10.0, 10.0, 0.0),
      max_linear_speed_present=True,
      max_linear_speed_m_s=5.0,
      max_angular_speed_present=True,
      max_angular_speed_rad_s=1.0,
  )


class ValidateTest(StateSpaceTestCase):

  def test_default_bounds_accept_finite_unit_states(self):
    self.assertIs(vss.validate(_fixture_b(), vss.VehicleStateBounds()), _OK)

  def test_structural_defects_with_default_bounds(self):
    bounds = vss.VehicleStateBounds()
    self.assertIs(
        vss.validate(_state(twist=(0, 0, 0, math.nan, 0, 0)), bounds),
        vss.StateSpaceError.NON_FINITE,
    )
    self.assertIs(
        vss.validate(_state(orientation=(0, 0, 0, 0.5)), bounds),
        vss.StateSpaceError.QUATERNION,
    )

  def test_inside_bounds_is_ok(self):
    state = _state(
        position=(1.0, -2.0, -3.0), twist=(3.0, 4.0, 0.0, 0.0, 0.6, 0.8)
    )
    self.assertIs(vss.validate(state, _engaged_bounds()), _OK)

  def test_position_limits_are_inclusive(self):
    bounds = _engaged_bounds()
    self.assertIs(
        vss.validate(_state(position=(-10.0, 10.0, 0.0)), bounds), _OK
    )
    for position in (
        (-10.0001, 0.0, 0.0),
        (0.0, 10.0001, 0.0),
        (0.0, 0.0, 0.0001),
        (0.0, 0.0, -20.0001),
    ):
      self.assertIs(
          vss.validate(_state(position=position), bounds),
          vss.StateSpaceError.BOUNDS,
          position,
      )

  def test_speed_limits_are_inclusive(self):
    bounds = _engaged_bounds()
    self.assertIs(
        vss.validate(_state(twist=(3.0, 4.0, 0.0, 0.0, 0.0, 1.0)), bounds), _OK
    )
    self.assertIs(
        vss.validate(_state(twist=(3.0, 4.0, 0.01, 0.0, 0.0, 0.0)), bounds),
        vss.StateSpaceError.BOUNDS,
    )
    self.assertIs(
        vss.validate(_state(twist=(0.0, 0.0, 0.0, 1.0, 0.0, 0.01)), bounds),
        vss.StateSpaceError.BOUNDS,
    )

  def test_absent_limits_do_not_constrain_that_axis(self):
    bounds = vss.VehicleStateBounds(
        max_linear_speed_present=True, max_linear_speed_m_s=1.0
    )
    state = _state(
        position=(1e6, -1e6, 1e6), twist=(0.0, 0.0, 0.0, 100.0, 100.0, 100.0)
    )
    self.assertIs(vss.validate(state, bounds), _OK)

  def test_zero_speed_limit_allows_only_zero_twist(self):
    bounds = vss.VehicleStateBounds(
        max_linear_speed_present=True, max_linear_speed_m_s=0.0
    )
    self.assertIs(vss.validate(_fixture_a(), bounds), _OK)
    self.assertIs(
        vss.validate(_state(twist=(1e-3, 0, 0, 0, 0, 0)), bounds),
        vss.StateSpaceError.BOUNDS,
    )

  def test_bad_bounds_configuration(self):
    state = _fixture_a()
    bad_error = vss.StateSpaceError.BAD_BOUNDS

    inverted = dataclasses.replace(
        _engaged_bounds(), position_min=(-10.0, 11.0, -20.0)
    )
    self.assertIs(vss.validate(state, inverted), bad_error)

    nan_limit = dataclasses.replace(
        _engaged_bounds(), position_max=(10.0, 10.0, math.nan)
    )
    self.assertIs(vss.validate(state, nan_limit), bad_error)

    for bad in (-1.0, math.nan, math.inf, -math.inf):
      linear = dataclasses.replace(_engaged_bounds(), max_linear_speed_m_s=bad)
      angular = dataclasses.replace(
          _engaged_bounds(), max_angular_speed_rad_s=bad
      )
      self.assertIs(vss.validate(state, linear), bad_error, bad)
      self.assertIs(vss.validate(state, angular), bad_error, bad)

  def test_disengaged_invalid_bounds_are_ignored(self):
    bounds = vss.VehicleStateBounds(
        position_min=(1.0, 1.0, 1.0),
        position_max=(-1.0, -1.0, -1.0),
        max_linear_speed_m_s=-5.0,
        max_angular_speed_rad_s=math.nan,
    )
    self.assertIs(vss.validate(_fixture_b(), bounds), _OK)

  def test_first_defect_wins(self):
    bounds = _engaged_bounds()
    state = _state(
        position=(100.0, math.nan, 0.0),
        orientation=(0, 0, 0, 2.0),
        twist=(100.0, 0, 0, 0, 0, 0),
    )
    bad = dataclasses.replace(bounds, max_angular_speed_rad_s=-1.0)
    self.assertIs(vss.validate(state, bad), vss.StateSpaceError.BAD_BOUNDS)
    self.assertIs(vss.validate(state, bounds), vss.StateSpaceError.NON_FINITE)
    state = dataclasses.replace(state, position=(100.0, 0.0, 0.0))
    self.assertIs(vss.validate(state, bounds), vss.StateSpaceError.QUATERNION)
    state = dataclasses.replace(state, orientation=(0.0, 0.0, 0.0, 1.0))
    self.assertIs(vss.validate(state, bounds), vss.StateSpaceError.BOUNDS)


class PropertyTest(StateSpaceTestCase):
  """Randomized checks with a fixed seed so failures reproduce."""

  ITERATIONS = 2000

  def setUp(self):
    super().setUp()
    self.rng = random.Random(0x1020304050607080)

  def random_unit_quaternion(self):
    q = tuple(self.rng.gauss(0.0, 1.0) for _ in range(4))
    norm = frame_policy.quaternion_norm(q)
    if norm < 1e-3:
      return (0.0, 0.0, 0.0, 1.0)
    return tuple(c / norm for c in q)

  def random_state(self, pos=100.0, lin=5.0, ang=2.0):
    uniform = self.rng.uniform
    root3 = math.sqrt(3.0)
    return _state(
        position=tuple(uniform(-pos, pos) for _ in range(3)),
        orientation=self.random_unit_quaternion(),
        twist=tuple(uniform(-lin, lin) / root3 for _ in range(3))
        + tuple(uniform(-ang, ang) / root3 for _ in range(3)),
    )

  def test_interpolate_output_is_finite_unit_and_valid(self):
    for _ in range(self.ITERATIONS):
      a, b = self.random_state(), self.random_state()
      result = vss.interpolate(a, b, self.rng.uniform(0.0, 1.0))
      self.assertIs(result.error, _OK)
      state = result.state
      self.assertTrue(frame_policy.is_finite_vec3(state.position))
      self.assertTrue(frame_policy.is_finite_quaternion(state.orientation))
      self.assertTrue(all(math.isfinite(c) for c in state.twist))
      self.assertAlmostEqual(
          frame_policy.quaternion_norm(state.orientation), 1.0, delta=1e-12
      )
      self.assertIs(vss.validate(state, vss.VehicleStateBounds()), _OK)

  def test_interpolate_endpoint_identity(self):
    for _ in range(self.ITERATIONS):
      a, b = self.random_state(), self.random_state()
      at_a = vss.interpolate(a, b, 0.0)
      at_b = vss.interpolate(a, b, 1.0)
      self.assertIs(at_a.error, _OK)
      self.assertIs(at_b.error, _OK)
      self.assert_state_near(at_a.state, a, 1e-12)
      self.assert_state_near(at_b.state, b, 1e-12)

  def test_interpolated_rotation_angle_is_linear_and_shortest(self):
    for _ in range(self.ITERATIONS):
      a = _state(orientation=self.random_unit_quaternion())
      b = _state(orientation=self.random_unit_quaternion())
      u = self.rng.uniform(0.0, 1.0)
      result = vss.interpolate(a, b, u)
      self.assertIs(result.error, _OK)
      mid = _state(orientation=result.state.orientation)
      total = vss.distance(a, b)
      self.assertLessEqual(total, math.pi + 1e-12)
      self.assertAlmostEqual(vss.distance(a, mid), u * total, delta=1e-7)
      self.assertAlmostEqual(
          vss.distance(mid, b), (1.0 - u) * total, delta=1e-7
      )

  def test_distance_is_symmetric_non_negative_and_zero_on_identity(self):
    for _ in range(self.ITERATIONS):
      a, b = self.random_state(), self.random_state()
      ab = vss.distance(a, b)
      self.assertGreaterEqual(ab, 0.0)
      self.assertTrue(math.isfinite(ab))
      self.assertEqual(ab, vss.distance(b, a))
      self.assertEqual(vss.distance(a, a), 0.0)

  def test_distance_satisfies_triangle_inequality(self):
    for _ in range(self.ITERATIONS):
      a, b, c = self.random_state(), self.random_state(), self.random_state()
      self.assertLessEqual(
          vss.distance(a, c), vss.distance(a, b) + vss.distance(b, c) + 1e-9
      )

  def test_interpolants_stay_inside_convex_bounds(self):
    bounds = vss.VehicleStateBounds(
        position_limits_present=True,
        position_min=(-100.0, -100.0, -100.0),
        position_max=(100.0, 100.0, 100.0),
        max_linear_speed_present=True,
        max_linear_speed_m_s=5.0,
        max_angular_speed_present=True,
        max_angular_speed_rad_s=2.0,
    )
    for _ in range(self.ITERATIONS):
      a = self.random_state(99.0, 4.9, 1.9)
      b = self.random_state(99.0, 4.9, 1.9)
      self.assertIs(vss.validate(a, bounds), _OK)
      self.assertIs(vss.validate(b, bounds), _OK)
      result = vss.interpolate(a, b, self.rng.uniform(0.0, 1.0))
      self.assertIs(result.error, _OK)
      self.assertIs(vss.validate(result.state, bounds), _OK)

  def test_validate_rejects_states_outside_bounds(self):
    bounds = vss.VehicleStateBounds(
        position_limits_present=True,
        position_min=(-10.0, -10.0, -10.0),
        position_max=(10.0, 10.0, 10.0),
    )
    for _ in range(self.ITERATIONS):
      state = self.random_state(20.0)
      inside = all(abs(c) <= 10.0 for c in state.position)
      expected = _OK if inside else vss.StateSpaceError.BOUNDS
      self.assertIs(vss.validate(state, bounds), expected)


if __name__ == "__main__":
  unittest.main()
