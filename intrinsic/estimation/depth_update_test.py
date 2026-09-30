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

"""Tests for the depth update. Mirrors the .cc test."""

import dataclasses
import math
import unittest

from intrinsic.estimation import depth_update
from intrinsic.estimation import eskf_state as eskf

_N = 15
_DP_Z = 2
_TOL = 1e-9
_CHI2 = 3.841
_NAN = float("nan")
_INF = float("inf")
_Status = depth_update.DepthUpdateStatus


def _diag_p(value, overrides=None):
  """P = value * I, with optional {(row, col): value} overrides."""
  v = [value if i == j else 0.0 for i in range(_N) for j in range(_N)]
  for (i, j), o in (overrides or {}).items():
    v[i * _N + j] = o
  return eskf.EskfCovariance(v)


def _sample(depth=10.0, r=0.01, valid=True, surface=0.0):
  return depth_update.DepthSample(
      depth_m=depth, R=r, valid=valid, free_surface_up_m=surface
  )


def _hover_x(z=-10.0):
  return eskf.EskfNominal(p_enu=(0.0, 0.0, z))


def _diag(p):
  return [p.at(i, i) for i in range(_N)]


def _golden_p():
  """Deliberately asymmetric, finite, deterministic."""
  v = [0.0] * (_N * _N)
  for i in range(_N):
    for j in range(_N):
      v[i * _N + j] = (
          0.05 + 0.01 * i if i == j else 0.002 * (((i * 7 + j * 3) % 5) - 2)
      )
  return eskf.EskfCovariance(v)


def _golden_x():
  return eskf.EskfNominal(
      p_enu=(1.0, 2.0, -3.0),
      q_wxyz=(0.9, 0.1, -0.2, 0.3),
      v_body=(0.5, -0.1, 0.2),
      b_a=(0.01, -0.02, 0.03),
      b_g=(0.001, 0.002, -0.003),
  )


_GOLDEN_Z = _sample(depth=3.6, r=0.02, surface=0.5)

# Independent numpy evaluation: K = sym(P) H^T S^-1, Joseph, right-error
# inject. Same constants as the C++ test.
_GOLDEN_D2 = 0.1111111111111113
_GOLDEN_NU = 0.1
_GOLDEN_NOMINAL = (
    0.9988888888888889,
    1.998888888888889,
    -3.077777777777778,
    0.9234940869278551,
    0.1023697926257516,
    -0.20582256189954848,
    0.30710937787725484,
    0.4988888888888889,
    -0.09555555555555556,
    0.1988888888888889,
    0.008888888888888889,
    -0.021111111111111112,
    0.028888888888888888,
    0.005444444444444448,
    0.0008888888888888881,
    -0.004111111111111112,
)
_GOLDEN_P_DIAG = (
    0.04998888888888889,
    0.05998888888888889,
    0.015555555555555559,
    0.07998888888888889,
    0.08998888888888888,
    0.09998888888888889,
    0.10998888888888889,
    0.11982222222222223,
    0.1299888888888889,
    0.1399888888888889,
    0.14998888888888892,
    0.1599888888888889,
    0.1698222222222222,
    0.1799888888888889,
    0.1899888888888889,
)


def _update(x, p, z, t=_CHI2):
  return depth_update.update_depth(x, p, z, t)


class DepthUpdateTest(unittest.TestCase):

  def _assert_unchanged(self, r, status, evaluated=False):
    self.assertEqual(r.status, status)
    self.assertEqual(r.evaluated, evaluated)
    self.assertFalse(r.accepted)
    self.assertIsNone(r.nominal)
    self.assertIsNone(r.P)
    self.assertIsNone(r.delta_x)
    self.assertEqual(r.diagnostics is not None, evaluated)

  def test_accept_hover(self):
    # p_z = -10, depth = 10, h = 10, nu = 0. S = 0.04 + 0.01 = 0.05,
    # K[dp_z] = -0.8, dp_z variance 0.04 -> 0.008.
    x = _hover_x()
    r = _update(x, _diag_p(0.04), _sample())
    self.assertEqual(r.status, _Status.OK_ACCEPT)
    self.assertTrue(r.evaluated)
    self.assertTrue(r.accepted)
    self.assertEqual(r.nominal.to_tuple(), x.to_tuple())
    self.assertEqual(r.delta_x.to_tuple(), (0.0,) * _N)
    diag = _diag(r.P)
    for i in range(_N):
      self.assertAlmostEqual(
          diag[i], 0.008 if i == _DP_Z else 0.04, delta=_TOL, msg=str(i)
      )
      for j in range(_N):
        if i != j:
          self.assertEqual(r.P.at(i, j), 0.0)
    self.assertEqual(r.diagnostics.mahalanobis_sq, 0.0)
    self.assertEqual(r.diagnostics.innovation_norm, 0.0)
    self.assertEqual(r.diagnostics.threshold, _CHI2)
    self.assertEqual(r.diagnostics.dof, 1)
    self.assertAlmostEqual(r.diagnostics.S[0], 0.05, delta=_TOL)

  def test_only_dp_z_variance_shrinks(self):
    r = _update(_hover_x(), _diag_p(0.04), _sample())
    self.assertLess(r.P.at(_DP_Z, _DP_Z), 0.04)
    for i in range(_N):
      if i != _DP_Z:
        self.assertEqual(r.P.at(i, i), 0.04)

  def test_deeper_reading_moves_p_down(self):
    # depth = 10.1, nu = 0.1, d^2 = 0.2, dp_z = -0.8 * 0.1 = -0.08.
    r = _update(_hover_x(), _diag_p(0.04), _sample(depth=10.1))
    self.assertEqual(r.status, _Status.OK_ACCEPT)
    self.assertAlmostEqual(r.diagnostics.mahalanobis_sq, 0.2, delta=_TOL)
    self.assertAlmostEqual(r.diagnostics.innovation_norm, 0.1, delta=_TOL)
    self.assertAlmostEqual(r.nominal.p_enu[2], -10.08, delta=_TOL)
    self.assertAlmostEqual(r.delta_x.to_tuple()[_DP_Z], -0.08, delta=_TOL)
    self.assertEqual(r.nominal.p_enu[0], 0.0)
    self.assertEqual(r.nominal.p_enu[1], 0.0)
    self.assertEqual(r.nominal.q_wxyz, (1.0, 0.0, 0.0, 0.0))

  def test_shallower_reading_moves_p_up(self):
    r = _update(_hover_x(), _diag_p(0.04), _sample(depth=9.9))
    self.assertAlmostEqual(r.nominal.p_enu[2], -9.92, delta=_TOL)

  def test_free_surface_offset(self):
    # Surface at +2 and p_z = -8 give the same 10 m depth as the hover.
    x = _hover_x(-8.0)
    r = _update(x, _diag_p(0.04), _sample(depth=10.0, surface=2.0))
    self.assertEqual(r.status, _Status.OK_ACCEPT)
    self.assertEqual(r.diagnostics.innovation_norm, 0.0)
    self.assertEqual(r.nominal.to_tuple(), x.to_tuple())
    # Surface at +2 and depth 10.1: h = 10, nu = 0.1.
    r = _update(x, _diag_p(0.04), _sample(depth=10.1, surface=2.0))
    self.assertAlmostEqual(r.nominal.p_enu[2], -8.08, delta=_TOL)

  def test_enu_ned_depth_sign(self):
    # Standard ENU -> NED axis swap: (n, e, d) = (y_enu, x_enu, -z_enu).
    p_enu = (3.0, -4.0, -10.0)
    p_ned = (p_enu[1], p_enu[0], -p_enu[2])
    depth = -p_enu[2]
    self.assertEqual(depth, 10.0)
    self.assertEqual(depth, p_ned[2])
    z = _sample(depth=p_ned[2])
    r = _update(eskf.EskfNominal(p_enu=p_enu), _diag_p(0.04), z)
    self.assertEqual(r.status, _Status.OK_ACCEPT)
    self.assertEqual(r.diagnostics.innovation_norm, 0.0)
    # Same point above the surface would be negative depth (still finite).
    above = (0.0, 0.0, 1.5)
    r = _update(
        eskf.EskfNominal(p_enu=above), _diag_p(0.04), _sample(depth=-1.5)
    )
    self.assertEqual(r.diagnostics.innovation_norm, 0.0)

  def test_correlated_state_is_injected(self):
    # K[i] = -P[i, dp_z] / 0.05 and nu = 0.1.
    p = _diag_p(
        0.04,
        {
            (0, 2): 0.02,
            (2, 0): 0.02,
            (4, 2): 0.01,
            (2, 4): 0.01,
            (9, 2): 0.005,
            (2, 9): 0.005,
            (13, 2): 0.0025,
            (2, 13): 0.0025,
        },
    )
    r = _update(_hover_x(), p, _sample(depth=10.1))
    self.assertEqual(r.status, _Status.OK_ACCEPT)
    n = r.nominal
    self.assertAlmostEqual(n.p_enu[0], -0.04, delta=_TOL)
    self.assertAlmostEqual(n.b_a[0], -0.01, delta=_TOL)
    self.assertAlmostEqual(n.b_g[1], -0.005, delta=_TOL)
    # dtheta_y = -0.02 -> q = normalize(1, 0, -0.01, 0).
    norm = math.sqrt(1.0 + 0.01 * 0.01)
    self.assertAlmostEqual(n.q_wxyz[0], 1.0 / norm, delta=_TOL)
    self.assertAlmostEqual(n.q_wxyz[2], -0.01 / norm, delta=_TOL)
    self.assertAlmostEqual(n.q_wxyz[1], 0.0, delta=_TOL)
    self.assertAlmostEqual(n.q_wxyz[3], 0.0, delta=_TOL)
    self.assertAlmostEqual(
        math.sqrt(sum(c * c for c in n.q_wxyz)), 1.0, delta=_TOL
    )
    self.assertAlmostEqual(r.delta_x.to_tuple()[4], -0.02, delta=_TOL)

  def test_covariance_is_symmetric_and_does_not_grow(self):
    r = _update(_golden_x(), _golden_p(), _GOLDEN_Z)
    self.assertEqual(r.status, _Status.OK_ACCEPT)
    prior = _golden_p()
    for i in range(_N):
      for j in range(_N):
        self.assertEqual(r.P.at(i, j), r.P.at(j, i))
      self.assertLessEqual(r.P.at(i, i), prior.at(i, i) + 1e-15)

  def test_asymmetric_p_matches_its_symmetrization(self):
    raw = _golden_p()
    sym = eskf.EskfCovariance(
        [
            0.5 * (raw.at(i, j) + raw.at(j, i))
            for i in range(_N)
            for j in range(_N)
        ]
    )
    a = _update(_golden_x(), raw, _GOLDEN_Z)
    b = _update(_golden_x(), sym, _GOLDEN_Z)
    self.assertEqual(a.nominal.to_tuple(), b.nominal.to_tuple())
    self.assertEqual(a.P.row_major, b.P.row_major)

  def test_outlier_reject_leaves_everything_unchanged(self):
    x = _hover_x()
    p = _diag_p(0.04)
    x_before = x.to_tuple()
    p_before = p.row_major
    r = _update(x, p, _sample(depth=11.0))
    self._assert_unchanged(r, _Status.OK_REJECT, evaluated=True)
    self.assertAlmostEqual(r.diagnostics.mahalanobis_sq, 20.0, delta=_TOL)
    self.assertEqual(r.diagnostics.dof, 1)
    self.assertEqual(x.to_tuple(), x_before)
    self.assertEqual(p.row_major, p_before)

  def test_threshold_boundary(self):
    z = _sample(depth=10.1)
    self.assertEqual(
        _update(_hover_x(), _diag_p(0.04), z, 0.2 + 1e-6).status,
        _Status.OK_ACCEPT,
    )
    self.assertEqual(
        _update(_hover_x(), _diag_p(0.04), z, 0.2 - 1e-6).status,
        _Status.OK_REJECT,
    )

  def test_invalid_flag(self):
    self._assert_unchanged(
        _update(_hover_x(), _diag_p(0.04), _sample(valid=False)),
        _Status.SKIPPED_INVALID,
    )

  def test_invalid_flag_wins_over_bad_contents(self):
    self._assert_unchanged(
        _update(
            _hover_x(),
            _diag_p(0.04),
            _sample(depth=_NAN, r=_NAN, valid=False),
        ),
        _Status.SKIPPED_INVALID,
    )

  def test_non_finite_depth_surface_or_r(self):
    for z in (
        _sample(depth=_NAN),
        _sample(depth=_INF),
        _sample(depth=-_INF),
        _sample(surface=_NAN),
        _sample(surface=_INF),
        _sample(r=_NAN),
        _sample(r=_INF),
    ):
      self._assert_unchanged(
          _update(_hover_x(), _diag_p(0.04), z), _Status.SKIPPED_INVALID
      )

  def test_non_positive_r_is_singular(self):
    for r in (0.0, -0.01, -1e-300, 1e-12, 1e-13):
      self._assert_unchanged(
          _update(_hover_x(), _diag_p(0.04), _sample(r=r)), _Status.SINGULAR
      )

  def test_r_just_above_floor_evaluates(self):
    self.assertEqual(
        _update(_hover_x(), _diag_p(0.04), _sample(r=2e-12)).status,
        _Status.OK_ACCEPT,
    )

  def test_invalid_threshold(self):
    for t in (0.0, -1.0, _NAN, _INF):
      self._assert_unchanged(
          _update(_hover_x(), _diag_p(0.04), _sample(), t),
          _Status.SKIPPED_INVALID,
      )

  def test_invalid_p(self):
    self._assert_unchanged(
        _update(_hover_x(), eskf.EskfCovariance(), _sample()),
        _Status.SKIPPED_INVALID,
    )
    self._assert_unchanged(
        _update(_hover_x(), _diag_p(0.04, {(3, 4): _NAN}), _sample()),
        _Status.SKIPPED_INVALID,
    )

  def test_invalid_nominal_and_quaternion(self):
    bad = [
        eskf.EskfNominal(p_enu=(_NAN, 0.0, 0.0)),
        eskf.EskfNominal(p_enu=(0.0, 0.0, _INF)),
        eskf.EskfNominal(v_body=(0.0, _INF, 0.0)),
        eskf.EskfNominal(b_a=(0.0, 0.0, _NAN)),
        eskf.EskfNominal(b_g=(_INF, 0.0, 0.0)),
        eskf.EskfNominal(q_wxyz=(1.0, _NAN, 0.0, 0.0)),
        eskf.EskfNominal(q_wxyz=(0.0, 0.0, 0.0, 0.0)),
        eskf.EskfNominal(q_wxyz=(1e-13, 0.0, 0.0, 0.0)),
    ]
    for x in bad:
      self._assert_unchanged(
          _update(x, _diag_p(0.04), _sample()), _Status.SKIPPED_INVALID
      )

  def test_non_finite_update_leaves_inputs_unchanged(self):
    # d^2 = 4 / 2 = 2 passes the gate, but K[0] = -1e200 / 2 and the Joseph
    # product K[0] * P[2, 0] overflows.
    p = _diag_p(1.0, {(0, 2): 1e200, (2, 0): 1e200})
    x = _hover_x()
    r = _update(x, p, _sample(depth=12.0, r=1.0))
    self._assert_unchanged(r, _Status.NON_FINITE, evaluated=True)
    self.assertEqual(r.diagnostics.dof, 1)
    self.assertEqual(x.to_tuple(), _hover_x().to_tuple())

  def test_gate_overflow_is_non_finite(self):
    self._assert_unchanged(
        _update(
            _hover_x(),
            _diag_p(0.04, {(2, 2): 1.5e308}),
            _sample(r=1.5e308),
        ),
        _Status.NON_FINITE,
    )

  def test_depth_overflow_is_non_finite(self):
    self._assert_unchanged(
        _update(
            _hover_x(), _diag_p(0.04), _sample(depth=-1.7e308, surface=1.7e308)
        ),
        _Status.NON_FINITE,
    )

  def test_golden_case(self):
    r = _update(_golden_x(), _golden_p(), _GOLDEN_Z)
    self.assertEqual(r.status, _Status.OK_ACCEPT)
    self.assertAlmostEqual(r.diagnostics.mahalanobis_sq, _GOLDEN_D2, delta=_TOL)
    self.assertAlmostEqual(
        r.diagnostics.innovation_norm, _GOLDEN_NU, delta=_TOL
    )
    for i, want in enumerate(_GOLDEN_NOMINAL):
      self.assertAlmostEqual(
          r.nominal.to_tuple()[i], want, delta=_TOL, msg=str(i)
      )
    for i, want in enumerate(_GOLDEN_P_DIAG):
      self.assertAlmostEqual(r.P.at(i, i), want, delta=_TOL, msg=str(i))

  def test_inputs_are_not_modified(self):
    x = _golden_x()
    p = _golden_p()
    z = _GOLDEN_Z
    x_before = x.to_tuple()
    p_before = p.row_major
    z_before = dataclasses.astuple(z)
    _update(x, p, z)
    self.assertEqual(x.to_tuple(), x_before)
    self.assertEqual(p.row_major, p_before)
    self.assertEqual(dataclasses.astuple(z), z_before)

  def test_determinism(self):
    a = _update(_golden_x(), _golden_p(), _GOLDEN_Z)
    b = _update(_golden_x(), _golden_p(), _GOLDEN_Z)
    self.assertEqual(a.nominal.to_tuple(), b.nominal.to_tuple())
    self.assertEqual(a.P.row_major, b.P.row_major)
    self.assertEqual(a.delta_x.to_tuple(), b.delta_x.to_tuple())
    self.assertEqual(a.diagnostics.mahalanobis_sq, b.diagnostics.mahalanobis_sq)
    self.assertEqual(a.diagnostics.S, b.diagnostics.S)

  def test_no_altitude_or_dvl_symbols(self):
    names = [n.lower() for n in dir(depth_update)]
    names += [
        f.name.lower() for f in dataclasses.fields(depth_update.DepthSample)
    ]
    names += [s.name.lower() for s in _Status]
    for banned in ("altitude", "baro", "asl", "dvl", "water", "bottom"):
      for n in names:
        self.assertNotIn(banned, n, msg=n)


if __name__ == "__main__":
  unittest.main()
