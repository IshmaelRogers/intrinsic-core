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

"""Tests for the DVL bottom-track update. Mirrors the .cc test."""

import math
import unittest

from intrinsic.estimation import dvl_bottom_track_update as dvl
from intrinsic.estimation import eskf_state as eskf

_N = 15
_TOL = 1e-9
_CHI2 = 7.815
_NAN = float("nan")
_INF = float("inf")
_Status = dvl.DvlUpdateStatus


def _diag_p(value, overrides=None):
  """P = value * I, with optional {(row, col): value} overrides."""
  v = [value if i == j else 0.0 for i in range(_N) for j in range(_N)]
  for (i, j), o in (overrides or {}).items():
    v[i * _N + j] = o
  return eskf.EskfCovariance(v)


def _r_diag(value):
  return (value, 0.0, 0.0, 0.0, value, 0.0, 0.0, 0.0, value)


def _sample(v=(0.0, 0.0, 0.0), r=None, lock=True, valid=True):
  return dvl.DvlBottomTrackSample(
      velocity_body_m_s=v,
      R_body=_r_diag(0.01) if r is None else tuple(r),
      bottom_lock=lock,
      valid=valid,
  )


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
_GOLDEN_Z = (0.55, -0.12, 0.25)

# Independent numpy evaluation: K = sym(P) H^T S^-1, Joseph, right-error inject.
_GOLDEN_D2 = 0.03860645238264895
_GOLDEN_NOMINAL = (
    1.0005583105621352,
    1.9986182873297131,
    -2.998678051989587,
    0.9234058908586854,
    0.10196666905428795,
    -0.20512914674990052,
    0.3079714147175167,
    0.5428508170289345,
    -0.11761626070688205,
    0.24222961984665276,
    0.01055831056213513,
    -0.01944168943786487,
    0.028618287329713138,
    0.0023219480104131624,
    0.000943143535603439,
    -0.0024416894378648706,
)
_GOLDEN_P_DIAG = (
    0.049980021774181524,
    0.0598614784238409,
    0.06987602105831801,
    0.07988148283170414,
    0.08998002177418153,
    0.09998002177418154,
    0.01685309586108271,
    0.023918402828986506,
    0.020952066641589353,
    0.13998002177418153,
    0.14998002177418154,
    0.1598614784238409,
    0.169876021058318,
    0.17988148283170413,
    0.18998002177418152,
)


class DvlBottomTrackUpdateTest(unittest.TestCase):

  def _assert_unchanged(self, result, status, evaluated=False):
    self.assertEqual(result.status, status)
    self.assertEqual(result.evaluated, evaluated)
    self.assertFalse(result.accepted)
    self.assertIsNone(result.nominal)
    self.assertIsNone(result.P)
    self.assertIsNone(result.delta_x)
    self.assertEqual(result.diagnostics is not None, evaluated)

  def test_accept_hover_zero_innovation(self):
    # S = 0.04 + 0.01 = 0.05, K = 0.8 on dv, dv variance 0.04 -> 0.008.
    x = eskf.EskfNominal()
    p = _diag_p(0.04)
    result = dvl.update_dvl_bottom_track(x, p, _sample(), _CHI2)
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

  def test_accept_nonzero_innovation(self):
    # nu = (0.1, 0, 0), d^2 = 0.01 / 0.05 = 0.2, dv = 0.8 * 0.1 = 0.08.
    result = dvl.update_dvl_bottom_track(
        eskf.EskfNominal(), _diag_p(0.04), _sample(v=(0.1, 0.0, 0.0)), _CHI2
    )
    self.assertEqual(result.status, _Status.OK_ACCEPT)
    self.assertAlmostEqual(result.diagnostics.mahalanobis_sq, 0.2, delta=_TOL)
    self.assertAlmostEqual(result.diagnostics.innovation_norm, 0.1, delta=_TOL)
    self.assertAlmostEqual(result.nominal.v_body[0], 0.08, delta=_TOL)
    self.assertAlmostEqual(result.nominal.v_body[1], 0.0, delta=_TOL)
    self.assertEqual(result.nominal.q_wxyz, (1.0, 0.0, 0.0, 0.0))
    self.assertAlmostEqual(result.P.at(6, 6), 0.008, delta=_TOL)

  def test_correlated_state_is_injected(self):
    # K[i, 0] = P[i, 6] / 0.05 and nu_x = 0.1.
    p = _diag_p(
        0.04,
        {
            (0, 6): 0.02,
            (6, 0): 0.02,
            (4, 6): 0.01,
            (6, 4): 0.01,
            (9, 6): 0.005,
            (6, 9): 0.005,
            (13, 6): 0.0025,
            (6, 13): 0.0025,
        },
    )
    result = dvl.update_dvl_bottom_track(
        eskf.EskfNominal(), p, _sample(v=(0.1, 0.0, 0.0)), _CHI2
    )
    self.assertEqual(result.status, _Status.OK_ACCEPT)
    n = result.nominal
    self.assertAlmostEqual(n.p_enu[0], 0.04, delta=_TOL)
    self.assertAlmostEqual(n.b_a[0], 0.01, delta=_TOL)
    self.assertAlmostEqual(n.b_g[1], 0.005, delta=_TOL)
    # dtheta_y = 0.02 -> q = normalize(1, 0, 0.01, 0).
    norm = math.sqrt(1.0 + 0.01**2)
    self.assertAlmostEqual(n.q_wxyz[0], 1.0 / norm, delta=_TOL)
    self.assertAlmostEqual(n.q_wxyz[2], 0.01 / norm, delta=_TOL)
    self.assertAlmostEqual(n.q_wxyz[1], 0.0, delta=_TOL)
    self.assertAlmostEqual(n.q_wxyz[3], 0.0, delta=_TOL)
    self.assertAlmostEqual(
        math.sqrt(sum(c * c for c in n.q_wxyz)), 1.0, delta=_TOL
    )
    self.assertAlmostEqual(result.delta_x.dtheta[1], 0.02, delta=_TOL)

  def test_covariance_is_symmetric_and_does_not_grow(self):
    result = dvl.update_dvl_bottom_track(
        _golden_x(), _golden_p(), _sample(_GOLDEN_Z, _GOLDEN_R), _CHI2
    )
    self.assertEqual(result.status, _Status.OK_ACCEPT)
    prior_sym = _golden_p()
    for i in range(_N):
      for j in range(_N):
        self.assertEqual(result.P.at(i, j), result.P.at(j, i))
      prior = prior_sym.at(i, i)
      self.assertLessEqual(result.P.at(i, i), prior + 1e-15)

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
    a = dvl.update_dvl_bottom_track(_golden_x(), raw, z, _CHI2)
    b = dvl.update_dvl_bottom_track(_golden_x(), sym, z, _CHI2)
    self.assertEqual(a.nominal, b.nominal)
    self.assertEqual(a.P.row_major, b.P.row_major)

  def test_deep_reject_leaves_everything_unchanged(self):
    x = eskf.EskfNominal()
    p = _diag_p(0.04)
    x_before, p_before = x, eskf.EskfCovariance(p.row_major)
    result = dvl.update_dvl_bottom_track(
        x, p, _sample(v=(1.0, 0.0, 0.0)), _CHI2
    )
    self._assert_unchanged(result, _Status.OK_REJECT, evaluated=True)
    self.assertAlmostEqual(result.diagnostics.mahalanobis_sq, 20.0, delta=_TOL)
    self.assertEqual(result.diagnostics.dof, 3)
    self.assertEqual(x, x_before)
    self.assertEqual(p.row_major, p_before.row_major)

  def test_threshold_boundary(self):
    z = _sample(v=(0.1, 0.0, 0.0))
    args = (eskf.EskfNominal(), _diag_p(0.04), z)
    self.assertEqual(
        dvl.update_dvl_bottom_track(*args, 0.2 + 1e-6).status,
        _Status.OK_ACCEPT,
    )
    self.assertEqual(
        dvl.update_dvl_bottom_track(*args, 0.2 - 1e-6).status,
        _Status.OK_REJECT,
    )

  def test_lock_loss_and_invalid_health(self):
    for lock, valid in ((False, True), (True, False), (False, False)):
      result = dvl.update_dvl_bottom_track(
          eskf.EskfNominal(),
          _diag_p(0.04),
          _sample(v=(0.1, 0.0, 0.0), lock=lock, valid=valid),
          _CHI2,
      )
      self._assert_unchanged(result, _Status.SKIPPED_LOCK_LOSS)

  def test_lock_loss_wins_over_bad_sample_contents(self):
    result = dvl.update_dvl_bottom_track(
        eskf.EskfNominal(),
        _diag_p(0.04),
        _sample(v=(_NAN, 0.0, 0.0), r=[_NAN] * 9, lock=False),
        _CHI2,
    )
    self._assert_unchanged(result, _Status.SKIPPED_LOCK_LOSS)

  def test_invalid_velocity(self):
    for v in ((_NAN, 0.0, 0.0), (0.0, _INF, 0.0), (0.0, 0.0, -_INF)):
      result = dvl.update_dvl_bottom_track(
          eskf.EskfNominal(), _diag_p(0.04), _sample(v=v), _CHI2
      )
      self._assert_unchanged(result, _Status.SKIPPED_INVALID)

  def test_invalid_r(self):
    bad_nan = list(_r_diag(0.01))
    bad_nan[4] = _NAN
    bad_inf = list(_r_diag(0.01))
    bad_inf[0] = _INF
    asym = list(_r_diag(0.01))
    asym[1] = 1e-11
    asym_big = list(_r_diag(0.01))
    asym_big[2] = 0.25
    asym_big[6] = -0.25
    for r in (bad_nan, bad_inf, asym, asym_big):
      result = dvl.update_dvl_bottom_track(
          eskf.EskfNominal(), _diag_p(0.04), _sample(r=r), _CHI2
      )
      self._assert_unchanged(result, _Status.SKIPPED_INVALID)

  def test_r_asymmetry_inside_tolerance_evaluates(self):
    r = list(_r_diag(0.01))
    r[1] = 5e-13
    result = dvl.update_dvl_bottom_track(
        eskf.EskfNominal(), _diag_p(0.04), _sample(r=r), _CHI2
    )
    self.assertEqual(result.status, _Status.OK_ACCEPT)

  def test_invalid_threshold(self):
    for t in (0.0, -1.0, _NAN, _INF):
      result = dvl.update_dvl_bottom_track(
          eskf.EskfNominal(), _diag_p(0.04), _sample(), t
      )
      self._assert_unchanged(result, _Status.SKIPPED_INVALID)

  def test_invalid_p(self):
    unknown = dvl.update_dvl_bottom_track(
        eskf.EskfNominal(), eskf.EskfCovariance(), _sample(), _CHI2
    )
    self._assert_unchanged(unknown, _Status.SKIPPED_INVALID)
    non_finite = dvl.update_dvl_bottom_track(
        eskf.EskfNominal(), _diag_p(0.04, {(3, 4): _NAN}), _sample(), _CHI2
    )
    self._assert_unchanged(non_finite, _Status.SKIPPED_INVALID)

  def test_invalid_nominal_and_quaternion(self):
    bad = (
        eskf.EskfNominal(p_enu=(_NAN, 0.0, 0.0)),
        eskf.EskfNominal(v_body=(0.0, _INF, 0.0)),
        eskf.EskfNominal(b_a=(0.0, 0.0, _NAN)),
        eskf.EskfNominal(b_g=(_INF, 0.0, 0.0)),
        eskf.EskfNominal(q_wxyz=(1.0, _NAN, 0.0, 0.0)),
        eskf.EskfNominal(q_wxyz=(0.0, 0.0, 0.0, 0.0)),
        eskf.EskfNominal(q_wxyz=(1e-13, 0.0, 0.0, 0.0)),
    )
    for x in bad:
      result = dvl.update_dvl_bottom_track(x, _diag_p(0.04), _sample(), _CHI2)
      self._assert_unchanged(result, _Status.SKIPPED_INVALID)

  def test_singular_r(self):
    indefinite = (1.0, 2.0, 0.0, 2.0, 1.0, 0.0, 0.0, 0.0, 1.0)
    for r in ((0.0,) * 9, indefinite, _r_diag(1e-12)):
      result = dvl.update_dvl_bottom_track(
          eskf.EskfNominal(), _diag_p(0.04), _sample(r=r), _CHI2
      )
      self._assert_unchanged(result, _Status.SINGULAR)

  def test_r_pivot_just_above_floor_evaluates(self):
    result = dvl.update_dvl_bottom_track(
        eskf.EskfNominal(), _diag_p(0.04), _sample(r=_r_diag(2e-12)), _CHI2
    )
    self.assertEqual(result.status, _Status.OK_ACCEPT)

  def test_non_finite_update_leaves_inputs_unchanged(self):
    # d^2 = 4 / 2 = 2 passes the gate, but K R K^T and the Joseph product
    # overflow: K[0] = 1e200 / 2, and K[0] * P[6, 0] = 5e399.
    p = _diag_p(1.0, {(0, 6): 1e200, (6, 0): 1e200})
    x = eskf.EskfNominal()
    result = dvl.update_dvl_bottom_track(
        x, p, _sample(v=(2.0, 0.0, 0.0), r=_r_diag(1.0)), _CHI2
    )
    self._assert_unchanged(result, _Status.NON_FINITE, evaluated=True)
    self.assertEqual(result.diagnostics.dof, 3)
    self.assertEqual(x, eskf.EskfNominal())

  def test_gate_overflow_is_non_finite(self):
    p = _diag_p(0.04, {(6, 6): 1.5e308})
    result = dvl.update_dvl_bottom_track(
        eskf.EskfNominal(), p, _sample(r=_r_diag(1.5e308)), _CHI2
    )
    self._assert_unchanged(result, _Status.NON_FINITE)

  def test_golden_case(self):
    result = dvl.update_dvl_bottom_track(
        _golden_x(), _golden_p(), _sample(_GOLDEN_Z, _GOLDEN_R), _CHI2
    )
    self.assertEqual(result.status, _Status.OK_ACCEPT)
    self.assertAlmostEqual(
        result.diagnostics.mahalanobis_sq, _GOLDEN_D2, delta=_TOL
    )
    got = result.nominal.to_tuple()
    for a, b in zip(got, _GOLDEN_NOMINAL):
      self.assertAlmostEqual(a, b, delta=_TOL)
    for a, b in zip(_diag(result.P), _GOLDEN_P_DIAG):
      self.assertAlmostEqual(a, b, delta=_TOL)

  def test_inputs_are_not_modified(self):
    x, p = _golden_x(), _golden_p()
    z = _sample(_GOLDEN_Z, _GOLDEN_R)
    p_before = p.row_major
    dvl.update_dvl_bottom_track(x, p, z, _CHI2)
    self.assertEqual(x, _golden_x())
    self.assertEqual(p.row_major, p_before)
    self.assertEqual(z, _sample(_GOLDEN_Z, _GOLDEN_R))

  def test_determinism(self):
    args = (_golden_x(), _golden_p(), _sample(_GOLDEN_Z, _GOLDEN_R), _CHI2)
    a = dvl.update_dvl_bottom_track(*args)
    b = dvl.update_dvl_bottom_track(*args)
    self.assertEqual(a.nominal, b.nominal)
    self.assertEqual(a.P.row_major, b.P.row_major)
    self.assertEqual(a.diagnostics, b.diagnostics)

  def test_no_water_track_surface(self):
    names = [n.lower() for n in dir(dvl)]
    self.assertFalse(any("water" in n for n in names))
    fields = [f.lower() for f in dvl.DvlBottomTrackSample.__dataclass_fields__]
    self.assertFalse(any("water" in f for f in fields))


if __name__ == "__main__":
  unittest.main()
