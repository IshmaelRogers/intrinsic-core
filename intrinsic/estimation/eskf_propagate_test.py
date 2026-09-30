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

"""Tests for ESKF nominal propagation. Mirrors eskf_propagate_test.cc."""

import math
import unittest

from intrinsic.estimation import eskf_propagate as prop
from intrinsic.estimation import eskf_state as eskf

_TOL = 1e-9
_NAN = float("nan")
_INF = float("inf")


def _hover_imu(q, omega=(0.0, 0.0, 0.0)):
  """a = -R^T g, so the specific force cancels gravity in the body frame."""
  r = prop.rotation_body_to_world(q)
  g_body = prop._mat_t_vec(r, prop.GRAVITY_ENU)
  return prop.ImuSample(
      accel_m_s2=tuple(-c for c in g_body), gyro_rad_s=tuple(omega)
  )


def _assert_near(test, actual, expected, tol=_TOL):
  test.assertEqual(len(actual), len(expected))
  for a, e in zip(actual, expected):
    test.assertAlmostEqual(a, e, delta=tol)


class EskfPropagateTest(unittest.TestCase):

  def test_gravity_constant(self):
    self.assertEqual(prop.kGravityEnu, (0.0, 0.0, -9.80665))

  def test_stationary_level_is_unchanged(self):
    x = eskf.EskfNominal(p_enu=(1.0, -2.0, 3.0))
    for dt in (1e-4, 0.01, 0.5, 1.0):
      r = prop.propagate_nominal(x, _hover_imu(x.q_wxyz), dt)
      self.assertTrue(r.ok)
      self.assertEqual(r.status, prop.PropagateStatus.OK)
      _assert_near(self, r.nominal.to_tuple(), x.to_tuple())

  def test_stationary_tilted_is_unchanged(self):
    s = math.sqrt(0.5)
    for q in ((s, 0.0, 0.0, s), (0.9, 0.1, -0.2, 0.3)):
      x = eskf.EskfNominal(q_wxyz=q)
      r = prop.propagate_nominal(x, _hover_imu(q), 0.02)
      self.assertTrue(r.ok)
      _assert_near(self, r.nominal.p_enu, x.p_enu)
      _assert_near(self, r.nominal.v_body, x.v_body)
      # Unit-normalized input is a fixed point. The 0.9... input is not unit
      # length and renormalizes, so compare against its normalized form.
      n = math.sqrt(sum(c * c for c in q))
      _assert_near(self, r.nominal.q_wxyz, tuple(c / n for c in q))

  def test_constant_rate_about_body_z(self):
    wz, dt = 0.5, 0.01
    x = eskf.EskfNominal()
    r = prop.propagate_nominal(x, _hover_imu(x.q_wxyz, (0.0, 0.0, wz)), dt)
    self.assertTrue(r.ok)
    _assert_near(self, r.nominal.p_enu, (0.0, 0.0, 0.0))
    _assert_near(self, r.nominal.v_body, (0.0, 0.0, 0.0))
    n = math.sqrt(1.0 + (0.5 * wz * dt) ** 2)
    _assert_near(self, r.nominal.q_wxyz, (1.0 / n, 0.0, 0.0, 0.5 * wz * dt / n))
    q = r.nominal.q_wxyz
    yaw = 2.0 * math.atan2(q[3], q[0])
    self.assertAlmostEqual(yaw, wz * dt, delta=1e-7)

  def test_constant_rate_integrated_yaw(self):
    wz, dt, steps = 0.5, 0.001, 1000
    x = eskf.EskfNominal()
    for _ in range(steps):
      r = prop.propagate_nominal(x, _hover_imu(x.q_wxyz, (0.0, 0.0, wz)), dt)
      self.assertTrue(r.ok)
      x = r.nominal
    yaw = 2.0 * math.atan2(x.q_wxyz[3], x.q_wxyz[0])
    self.assertAlmostEqual(yaw, wz * dt * steps, delta=1e-4)
    _assert_near(self, x.p_enu, (0.0, 0.0, 0.0), 1e-9)

  def test_invalid_dt(self):
    x = eskf.EskfNominal()
    imu = _hover_imu(x.q_wxyz)
    for dt in (0.0, -0.01, _NAN, _INF, -_INF, 1.0000001, 2.0):
      r = prop.propagate_nominal(x, imu, dt)
      self.assertFalse(r.ok)
      self.assertEqual(r.status, prop.PropagateStatus.INVALID_DT)
      self.assertIsNone(r.nominal)

  def test_invalid_imu(self):
    x = eskf.EskfNominal()
    good = _hover_imu(x.q_wxyz)
    for bad in (_NAN, _INF, -_INF):
      for i in range(3):
        a = list(good.accel_m_s2)
        a[i] = bad
        g = list(good.gyro_rad_s)
        g[i] = bad
        for imu in (
            prop.ImuSample(tuple(a), good.gyro_rad_s),
            prop.ImuSample(good.accel_m_s2, tuple(g)),
        ):
          r = prop.propagate_nominal(x, imu, 0.01)
          self.assertFalse(r.ok)
          self.assertEqual(r.status, prop.PropagateStatus.INVALID_IMU)
          self.assertIsNone(r.nominal)

  def test_invalid_dt_wins_over_invalid_imu(self):
    x = eskf.EskfNominal()
    imu = prop.ImuSample((_NAN, 0.0, 0.0), (0.0, 0.0, 0.0))
    r = prop.propagate_nominal(x, imu, 0.0)
    self.assertEqual(r.status, prop.PropagateStatus.INVALID_DT)

  def test_near_zero_quaternion(self):
    imu = prop.ImuSample()
    for q in (
        (0.0, 0.0, 0.0, 0.0),
        (1e-13, 0.0, 0.0, 0.0),
        (0.0, 0.0, 0.0, -1e-13),
    ):
      r = prop.propagate_nominal(eskf.EskfNominal(q_wxyz=q), imu, 0.01)
      self.assertFalse(r.ok)
      self.assertEqual(r.status, prop.PropagateStatus.INVALID_QUATERNION)
      self.assertIsNone(r.nominal)

  def test_non_finite_state(self):
    imu = prop.ImuSample()
    for x in (
        eskf.EskfNominal(p_enu=(_NAN, 0.0, 0.0)),
        eskf.EskfNominal(q_wxyz=(1.0, _INF, 0.0, 0.0)),
        eskf.EskfNominal(v_body=(0.0, _NAN, 0.0)),
        eskf.EskfNominal(b_a=(0.0, 0.0, _INF)),
        eskf.EskfNominal(b_g=(_NAN, 0.0, 0.0)),
    ):
      r = prop.propagate_nominal(x, imu, 0.01)
      self.assertFalse(r.ok)
      self.assertEqual(r.status, prop.PropagateStatus.NON_FINITE_STATE)
      self.assertIsNone(r.nominal)

  def test_overflow_output_is_non_finite_state(self):
    x = eskf.EskfNominal(v_body=(1e308, 0.0, 0.0))
    imu = prop.ImuSample((1e308, 0.0, 0.0), (0.0, 0.0, 0.0))
    r = prop.propagate_nominal(x, imu, 1.0)
    self.assertFalse(r.ok)
    self.assertEqual(r.status, prop.PropagateStatus.NON_FINITE_STATE)
    self.assertIsNone(r.nominal)

  def test_velocity_and_position_use_pre_update_attitude(self):
    # q = identity, v = (1, 0, 0), w = (0, 0, 1), hover. w x v = (0, 1, 0).
    dt = 0.1
    x = eskf.EskfNominal(v_body=(1.0, 0.0, 0.0))
    r = prop.propagate_nominal(x, _hover_imu(x.q_wxyz, (0.0, 0.0, 1.0)), dt)
    self.assertTrue(r.ok)
    _assert_near(self, r.nominal.v_body, (1.0, -dt, 0.0))
    _assert_near(self, r.nominal.p_enu, (dt, 0.0, 0.0))

  def test_biases_are_subtracted_and_held(self):
    b_a, b_g = (0.1, -0.2, 0.3), (0.01, 0.02, -0.03)
    x = eskf.EskfNominal(b_a=b_a, b_g=b_g)
    imu = prop.ImuSample(
        accel_m_s2=(b_a[0], b_a[1], 9.80665 + b_a[2]), gyro_rad_s=b_g
    )
    r = prop.propagate_nominal(x, imu, 0.05)
    self.assertTrue(r.ok)
    self.assertEqual(r.nominal.b_a, b_a)
    self.assertEqual(r.nominal.b_g, b_g)
    _assert_near(self, r.nominal.v_body, (0.0, 0.0, 0.0))
    _assert_near(self, r.nominal.q_wxyz, (1.0, 0.0, 0.0, 0.0))

  def test_dt_upper_bound_is_inclusive(self):
    x = eskf.EskfNominal()
    r = prop.propagate_nominal(x, _hover_imu(x.q_wxyz), 1.0)
    self.assertTrue(r.ok)

  def test_deterministic_and_golden(self):
    x = eskf.EskfNominal(
        p_enu=(1.0, 2.0, 3.0),
        q_wxyz=(0.9, 0.1, -0.2, 0.3),
        v_body=(0.5, -0.25, 0.1),
        b_a=(0.01, -0.02, 0.03),
        b_g=(0.001, 0.002, -0.003),
    )
    imu = prop.ImuSample((0.3, -0.4, 9.5), (0.05, -0.1, 0.2))
    first = prop.propagate_nominal(x, imu, 0.01)
    second = prop.propagate_nominal(x, imu, 0.01)
    self.assertEqual(first, second)
    _assert_near(
        self,
        first.nominal.to_tuple(),
        (
            1.004842105263158,
            2.000342105263158,
            3.0029473684210526,
            0.9229376970626407,
            0.10277269512801493,
            -0.20569518299429798,
            0.3087284764477931,
            0.45913878421052634,
            -0.2609596736842105,
            0.10656878947368423,
            0.01,
            -0.02,
            0.03,
            0.001,
            0.002,
            -0.003,
        ),
    )


if __name__ == "__main__":
  unittest.main()
