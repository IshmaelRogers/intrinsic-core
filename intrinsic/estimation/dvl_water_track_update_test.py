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

"""Tests for the DVL water-track update. Mirrors the .cc test."""

import math
import os
import unittest

from intrinsic.estimation import dvl_water_track_update as dvl
from intrinsic.estimation import eskf_state as eskf

_N = 15
_TOL = 1e-9
_CHI2 = 7.815
_NAN = float("nan")
_INF = float("inf")
_Status = dvl.DvlWaterTrackStatus


def _diag_p(value, overrides=None):
  """P = value * I, with optional {(row, col): value} overrides."""
  v = [value if i == j else 0.0 for i in range(_N) for j in range(_N)]
  for (i, j), o in (overrides or {}).items():
    v[i * _N + j] = o
  return eskf.EskfCovariance(v)


def _r_diag(value):
  return (value, 0.0, 0.0, 0.0, value, 0.0, 0.0, 0.0, value)


def _sample(v=(0.0, 0.0, 0.0), r=None, valid=True):
  return dvl.DvlWaterTrackSample(
      velocity_body_m_s=v,
      R_body=_r_diag(0.01) if r is None else tuple(r),
      valid=valid,
  )


def _current(v=(0.0, 0.0, 0.0), present=True):
  return dvl.WaterCurrentEstimate(present=present, v_enu_m_s=v)


def _diag(p):
  return [p.at(i, i) for i in range(_N)]


def _golden_p():
  """Deliberately asymmetric, finite, deterministic."""
  return eskf.EskfCovariance(
      [
          0.05 + 0.01 * i if i == j else 0.002 * (((i * 7 + j * 3) % 5) - 2)
          for i in range(_N)
          for j in range(_N)
      ]
  )


def _golden_x():
  return eskf.EskfNominal(
      p_enu=(1.0, 2.0, -3.0),
      q_wxyz=(0.9, 0.1, -0.2, 0.3),
      v_body=(0.5, -0.1, 0.2),
      b_a=(0.01, -0.02, 0.03),
      b_g=(0.001, 0.002, -0.003),
  )


_GOLDEN_R = (0.02, 0.004, 0.0, 0.004, 0.03, 0.002, 0.0, 0.002, 0.025)
_GOLDEN_Z = (0.15, 0.25, -0.05)
_GOLDEN_C = (0.4, -0.3, 0.1)

# Independent numpy evaluation: c_b = R(q/|q|)^T c, H from -[c_b x] (checked
# against a central finite difference of h under a right-error perturbation),
# K = sym(P) H^T S^-1, Joseph, right-error inject.
_GOLDEN_D2 = 0.4719881583806437
_GOLDEN_S = (
    0.15395940166204986,
    0.014026326869806096,
    -0.0003772011080332405,
    0.014026326869806097,
    0.1536128753462604,
    0.00485063490304709,
    -0.0003772011080332404,
    0.004850634903047091,
    0.17945670470914127,
)
_GOLDEN_NOMINAL = (
    1.0003685806512679,
    2.0025635648950675,
    -2.999355088831938,
    0.9311518839827754,
    0.12870942214744951,
    -0.19092067948327526,
    0.2827354731041758,
    0.38113109515252125,
    -0.18385691717479807,
    0.05853281176318678,
    0.006613701232377509,
    -0.019631419348732172,
    0.03256356489506766,
    0.0016449111680622566,
    0.0018092420532247464,
    -0.006386298767622491,
)
_GOLDEN_P_DIAG = (
    0.0499941380243371,
    0.059907913801712745,
    0.06987733146632756,
    0.07004439083278782,
    0.0883869300385078,
    0.08337714480699841,
    0.030114374741023367,
    0.025420693106184777,
    0.033258788189523665,
    0.13997524540177425,
    0.14999413802433714,
    0.15990791380171274,
    0.16987733146632755,
    0.17994428989577507,
    0.18997524540177424,
)


class DvlWaterTrackUpdateTest(unittest.TestCase):

  def _assert_unchanged(self, result, status, evaluated=False):
    self.assertEqual(result.status, status)
    self.assertEqual(result.evaluated, evaluated)
    self.assertFalse(result.accepted)
    self.assertIsNone(result.nominal)
    self.assertIsNone(result.P)
    self.assertIsNone(result.delta_x)
    self.assertEqual(result.diagnostics is not None, evaluated)

  def _update(self, x, p, z, current, t=_CHI2):
    return dvl.update_dvl_water_track(x, p, z, current, t)

  def test_accept_hover_zero_current(self):
    # S = 0.04 + 0.01 = 0.05, K = 0.8 on dv, dv variance 0.04 -> 0.008.
    x = eskf.EskfNominal()
    result = self._update(x, _diag_p(0.04), _sample(), _current())
    self.assertEqual(result.status, _Status.OK_ACCEPT)
    self.assertTrue(result.evaluated)
    self.assertTrue(result.accepted)
    self.assertEqual(result.nominal, x)
    self.assertEqual(result.delta_x, eskf.EskfError())
    diag = _diag(result.P)
    for i in range(_N):
      expected = 0.008 if 6 <= i <= 8 else 0.04
      self.assertAlmostEqual(diag[i], expected, delta=_TOL)
    for i in range(_N):
      for j in range(_N):
        if i != j:
          self.assertEqual(result.P.at(i, j), 0.0)
    d = result.diagnostics
    self.assertEqual(d.mahalanobis_sq, 0.0)
    self.assertEqual(d.innovation_norm, 0.0)
    self.assertEqual(d.threshold, _CHI2)
    self.assertEqual(d.dof, 3)

  def test_nonzero_current_identity_attitude_zero_innovation(self):
    # c_b = c_enu = (0.5, 0, 0), h = (-0.5, 0, 0), z = h, so nu = 0.
    # H theta = [[0,0,0],[0,0,0.5],[0,-0.5,0]], S = diag(0.05, 0.06, 0.06).
    x = eskf.EskfNominal()
    result = self._update(
        x,
        _diag_p(0.04),
        _sample((-0.5, 0.0, 0.0)),
        _current((0.5, 0.0, 0.0)),
    )
    self.assertEqual(result.status, _Status.OK_ACCEPT)
    self.assertLess(abs(result.diagnostics.mahalanobis_sq), 1e-15)
    self.assertLess(result.diagnostics.innovation_norm, 1e-15)
    self.assertEqual(result.nominal, x)
    self.assertEqual(result.delta_x, eskf.EskfError())
    s = result.diagnostics.S
    for i, expected in enumerate((0.05, 0.06, 0.06)):
      self.assertAlmostEqual(s[i * 3 + i], expected, delta=_TOL)
    diag = _diag(result.P)
    expected = [0.04] * _N
    expected[6] = 0.04 - 0.04**2 / 0.05
    expected[7] = 0.04 - 0.04**2 / 0.06
    expected[8] = 0.04 - 0.04**2 / 0.06
    expected[4] = 0.04 - (0.04 * 0.5) ** 2 / 0.06
    expected[5] = 0.04 - (0.04 * 0.5) ** 2 / 0.06
    for i in range(_N):
      self.assertAlmostEqual(diag[i], expected[i], delta=_TOL, msg=str(i))

  def test_attitude_coupling_injects_dtheta(self):
    # nu = (0, 0.06, 0). K[dv_y] = 0.04 / 0.06, K[dtheta_z] = 0.02 / 0.06.
    result = self._update(
        eskf.EskfNominal(),
        _diag_p(0.04),
        _sample((-0.5, 0.06, 0.0)),
        _current((0.5, 0.0, 0.0)),
    )
    self.assertEqual(result.status, _Status.OK_ACCEPT)
    self.assertAlmostEqual(result.delta_x.dv[1], 0.04, delta=_TOL)
    self.assertAlmostEqual(result.delta_x.dtheta[2], 0.02, delta=_TOL)
    self.assertAlmostEqual(result.delta_x.dtheta[0], 0.0, delta=_TOL)
    self.assertAlmostEqual(result.nominal.v_body[1], 0.04, delta=_TOL)
    norm = math.sqrt(1.0 + 0.01**2)
    self.assertAlmostEqual(result.nominal.q_wxyz[0], 1.0 / norm, delta=_TOL)
    self.assertAlmostEqual(result.nominal.q_wxyz[3], 0.01 / norm, delta=_TOL)

  def test_current_is_rotated_into_body(self):
    # q = +90 deg about z: R maps body x to world y. c_enu = (0, 1, 0) is then
    # c_b = R^T c = (1, 0, 0), so z = -c_b gives nu = 0.
    s = math.sqrt(0.5)
    x = eskf.EskfNominal(q_wxyz=(s, 0.0, 0.0, s))
    result = self._update(
        x,
        _diag_p(0.04),
        _sample((-1.0, 0.0, 0.0)),
        _current((0.0, 1.0, 0.0)),
    )
    self.assertEqual(result.status, _Status.OK_ACCEPT)
    self.assertLess(result.diagnostics.innovation_norm, 1e-12)

  def test_missing_current_skips(self):
    x = _golden_x()
    p = _golden_p()
    x_before, p_before = x.to_tuple(), p.row_major
    cases = [
        _current((0.4, 0.0, 0.0), present=False),
        _current((_NAN, 0.0, 0.0)),
        _current((0.0, _INF, 0.0)),
        _current((0.0, 0.0, -_INF)),
    ]
    for current in cases:
      result = self._update(x, p, _sample(_GOLDEN_Z, _GOLDEN_R), current)
      self._assert_unchanged(result, _Status.SKIPPED_MISSING_CURRENT)
    self.assertEqual(x.to_tuple(), x_before)
    self.assertEqual(p.row_major, p_before)

  def test_missing_current_wins_over_bad_contents(self):
    result = self._update(
        eskf.EskfNominal(),
        _diag_p(0.04),
        _sample((_NAN, 0.0, 0.0), (_NAN,) * 9),
        _current(present=False),
        _NAN,
    )
    self._assert_unchanged(result, _Status.SKIPPED_MISSING_CURRENT)

  def test_default_current_is_missing(self):
    result = self._update(
        eskf.EskfNominal(),
        _diag_p(0.04),
        _sample(),
        dvl.WaterCurrentEstimate(),
    )
    self._assert_unchanged(result, _Status.SKIPPED_MISSING_CURRENT)

  def test_unused_current_covariance_does_not_matter(self):
    base = self._update(
        _golden_x(),
        _golden_p(),
        _sample(_GOLDEN_Z, _GOLDEN_R),
        _current(_GOLDEN_C),
    )
    other = self._update(
        _golden_x(),
        _golden_p(),
        _sample(_GOLDEN_Z, _GOLDEN_R),
        dvl.WaterCurrentEstimate(
            present=True, v_enu_m_s=_GOLDEN_C, R_current_enu=(_NAN,) * 9
        ),
    )
    self.assertEqual(base.nominal, other.nominal)
    self.assertEqual(base.P.row_major, other.P.row_major)

  def test_covariance_is_symmetric_and_does_not_grow(self):
    result = self._update(
        _golden_x(),
        _golden_p(),
        _sample(_GOLDEN_Z, _GOLDEN_R),
        _current(_GOLDEN_C),
    )
    self.assertEqual(result.status, _Status.OK_ACCEPT)
    prior = _golden_p()
    for i in range(_N):
      for j in range(_N):
        self.assertEqual(result.P.at(i, j), result.P.at(j, i))
      self.assertLessEqual(result.P.at(i, i), prior.at(i, i) + 1e-15)

  def test_asymmetric_p_matches_its_symmetrization(self):
    raw = _golden_p()
    sym = eskf.EskfCovariance(
        [
            0.5 * (raw.at(i, j) + raw.at(j, i))
            for i in range(_N)
            for j in range(_N)
        ]
    )
    z = _sample(_GOLDEN_Z, _GOLDEN_R)
    c = _current(_GOLDEN_C)
    a = self._update(_golden_x(), raw, z, c)
    b = self._update(_golden_x(), sym, z, c)
    self.assertEqual(a.nominal, b.nominal)
    self.assertEqual(a.P.row_major, b.P.row_major)

  def test_deep_reject_leaves_everything_unchanged(self):
    x = eskf.EskfNominal()
    p = _diag_p(0.04)
    x_before, p_before = x.to_tuple(), p.row_major
    result = self._update(
        x, p, _sample((1.0, 0.0, 0.0)), _current((0.5, 0.0, 0.0))
    )
    # nu = (1.5, 0, 0), S_xx = 0.05, d^2 = 2.25 / 0.05 = 45.
    self._assert_unchanged(result, _Status.OK_REJECT, evaluated=True)
    self.assertAlmostEqual(result.diagnostics.mahalanobis_sq, 45.0, delta=_TOL)
    self.assertEqual(result.diagnostics.dof, 3)
    self.assertEqual(x.to_tuple(), x_before)
    self.assertEqual(p.row_major, p_before)

  def test_threshold_boundary(self):
    z = _sample((0.1, 0.0, 0.0))
    c = _current()
    x, p = eskf.EskfNominal(), _diag_p(0.04)
    self.assertEqual(
        self._update(x, p, z, c, 0.2 + 1e-6).status, _Status.OK_ACCEPT
    )
    self.assertEqual(
        self._update(x, p, z, c, 0.2 - 1e-6).status, _Status.OK_REJECT
    )

  def test_invalid_sample_flag(self):
    result = self._update(
        eskf.EskfNominal(),
        _diag_p(0.04),
        _sample((0.1, 0.0, 0.0), valid=False),
        _current(),
    )
    self._assert_unchanged(result, _Status.SKIPPED_INVALID)

  def test_invalid_velocity(self):
    for v in ((_NAN, 0.0, 0.0), (0.0, _INF, 0.0), (0.0, 0.0, -_INF)):
      self._assert_unchanged(
          self._update(
              eskf.EskfNominal(), _diag_p(0.04), _sample(v), _current()
          ),
          _Status.SKIPPED_INVALID,
      )

  def test_invalid_r(self):
    bad_nan = list(_r_diag(0.01))
    bad_nan[4] = _NAN
    bad_inf = list(_r_diag(0.01))
    bad_inf[0] = _INF
    asym = list(_r_diag(0.01))
    asym[1] = 1e-11
    asym_big = list(_r_diag(0.01))
    asym_big[2], asym_big[6] = 0.25, -0.25
    for r in (bad_nan, bad_inf, asym, asym_big, (0.01,) * 8):
      self._assert_unchanged(
          self._update(
              eskf.EskfNominal(),
              _diag_p(0.04),
              _sample((0, 0, 0), r),
              _current(),
          ),
          _Status.SKIPPED_INVALID,
      )

  def test_r_asymmetry_inside_tolerance_evaluates(self):
    r = list(_r_diag(0.01))
    r[1] = 5e-13
    result = self._update(
        eskf.EskfNominal(), _diag_p(0.04), _sample((0, 0, 0), r), _current()
    )
    self.assertEqual(result.status, _Status.OK_ACCEPT)

  def test_invalid_threshold(self):
    for t in (0.0, -1.0, _NAN, _INF):
      self._assert_unchanged(
          self._update(
              eskf.EskfNominal(), _diag_p(0.04), _sample(), _current(), t
          ),
          _Status.SKIPPED_INVALID,
      )

  def test_invalid_p(self):
    self._assert_unchanged(
        self._update(
            eskf.EskfNominal(), eskf.EskfCovariance(), _sample(), _current()
        ),
        _Status.SKIPPED_INVALID,
    )
    self._assert_unchanged(
        self._update(
            eskf.EskfNominal(),
            _diag_p(0.04, {(3, 4): _NAN}),
            _sample(),
            _current(),
        ),
        _Status.SKIPPED_INVALID,
    )

  def test_invalid_nominal_and_quaternion(self):
    bad = [
        eskf.EskfNominal(p_enu=(_NAN, 0.0, 0.0)),
        eskf.EskfNominal(v_body=(0.0, _INF, 0.0)),
        eskf.EskfNominal(b_a=(0.0, 0.0, _NAN)),
        eskf.EskfNominal(b_g=(_INF, 0.0, 0.0)),
        eskf.EskfNominal(q_wxyz=(1.0, _NAN, 0.0, 0.0)),
        eskf.EskfNominal(q_wxyz=(0.0, 0.0, 0.0, 0.0)),
        eskf.EskfNominal(q_wxyz=(1e-13, 0.0, 0.0, 0.0)),
    ]
    for x in bad:
      self._assert_unchanged(
          self._update(x, _diag_p(0.04), _sample(), _current((1.0, 0.0, 0.0))),
          _Status.SKIPPED_INVALID,
      )

  def test_singular_r(self):
    zero = (0.0,) * 9
    indefinite = (1, 2, 0, 2, 1, 0, 0, 0, 1)
    for r in (zero, indefinite, _r_diag(1e-12)):
      self._assert_unchanged(
          self._update(
              eskf.EskfNominal(),
              _diag_p(0.04),
              _sample((0, 0, 0), r),
              _current(),
          ),
          _Status.SINGULAR,
      )

  def test_r_pivot_just_above_floor_evaluates(self):
    result = self._update(
        eskf.EskfNominal(),
        _diag_p(0.04),
        _sample((0, 0, 0), _r_diag(2e-12)),
        _current(),
    )
    self.assertEqual(result.status, _Status.OK_ACCEPT)

  def test_non_finite_update_leaves_inputs_unchanged(self):
    # d^2 = 4 / 2 = 2 passes the gate, but the Joseph product overflows:
    # K[0] = 1e200 / 2, and K[0] * P[6, 0] = 5e399.
    p = _diag_p(1.0, {(0, 6): 1e200, (6, 0): 1e200})
    result = self._update(
        eskf.EskfNominal(),
        p,
        _sample((2.0, 0.0, 0.0), _r_diag(1.0)),
        _current(),
    )
    self._assert_unchanged(result, _Status.NON_FINITE, evaluated=True)
    self.assertEqual(result.diagnostics.dof, 3)

  def test_gate_overflow_is_non_finite(self):
    self._assert_unchanged(
        self._update(
            eskf.EskfNominal(),
            _diag_p(0.04, {(6, 6): 1.5e308}),
            _sample((0, 0, 0), _r_diag(1.5e308)),
            _current(),
        ),
        _Status.NON_FINITE,
    )

  def test_huge_current_overflow_is_non_finite(self):
    self._assert_unchanged(
        self._update(
            eskf.EskfNominal(q_wxyz=(0.9, 0.1, -0.2, 0.3)),
            _diag_p(0.04),
            _sample(),
            _current((1.7e308, 1.7e308, 1.7e308)),
        ),
        _Status.NON_FINITE,
    )

  def test_golden_case(self):
    result = self._update(
        _golden_x(),
        _golden_p(),
        _sample(_GOLDEN_Z, _GOLDEN_R),
        _current(_GOLDEN_C),
    )
    self.assertEqual(result.status, _Status.OK_ACCEPT)
    self.assertAlmostEqual(
        result.diagnostics.mahalanobis_sq, _GOLDEN_D2, delta=_TOL
    )
    for got, want in zip(result.diagnostics.S, _GOLDEN_S):
      self.assertAlmostEqual(got, want, delta=_TOL)
    for got, want in zip(result.nominal.to_tuple(), _GOLDEN_NOMINAL):
      self.assertAlmostEqual(got, want, delta=_TOL)
    for got, want in zip(_diag(result.P), _GOLDEN_P_DIAG):
      self.assertAlmostEqual(got, want, delta=_TOL)

  def test_inputs_are_not_modified(self):
    x = _golden_x()
    p = _golden_p()
    z = _sample(_GOLDEN_Z, _GOLDEN_R)
    c = _current(_GOLDEN_C)
    x_before, p_before = x.to_tuple(), p.row_major
    self._update(x, p, z, c)
    self.assertEqual(x.to_tuple(), x_before)
    self.assertEqual(p.row_major, p_before)
    self.assertEqual(z, _sample(_GOLDEN_Z, _GOLDEN_R))
    self.assertEqual(c, _current(_GOLDEN_C))

  def test_determinism(self):
    z = _sample(_GOLDEN_Z, _GOLDEN_R)
    c = _current(_GOLDEN_C)
    a = self._update(_golden_x(), _golden_p(), z, c)
    b = self._update(_golden_x(), _golden_p(), z, c)
    self.assertEqual(a.nominal, b.nominal)
    self.assertEqual(a.P.row_major, b.P.row_major)
    self.assertEqual(a.diagnostics, b.diagnostics)

  def test_no_ground_referenced_velocity_api(self):
    names = [n.lower() for n in dir(dvl)]
    names += [
        f.name.lower() for f in dvl.dataclasses.fields(dvl.DvlWaterTrackSample)
    ]
    for name in names:
      self.assertNotIn("bottom", name)
      self.assertNotIn("lock", name)
    path = dvl.__file__
    with open(path, encoding="utf-8") as f:
      source = f.read().lower()
    self.assertNotIn("dvl_bottom_track", source)
    cc = os.path.splitext(path)[0] + ".cc"
    if os.path.exists(cc):
      with open(cc, encoding="utf-8") as f:
        cc_source = f.read().lower()
      self.assertNotIn("bottom", cc_source)


if __name__ == "__main__":
  unittest.main()
