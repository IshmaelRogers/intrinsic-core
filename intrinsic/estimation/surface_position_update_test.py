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

"""Tests for the surface position update. Mirrors the .cc test."""

import dataclasses
import math
import os
import unittest

from intrinsic.estimation import eskf_state as eskf
from intrinsic.estimation import surface_position_update as spu

_N = 15
_TOL = 1e-9
_CHI2 = 5.991
_NAN = float("nan")
_INF = float("inf")
_Status = spu.SurfacePositionUpdateStatus


def _diag_p(value, overrides=None):
  """P = value * I, with optional {(row, col): value} overrides."""
  v = [value if i == j else 0.0 for i in range(_N) for j in range(_N)]
  for (i, j), o in (overrides or {}).items():
    v[i * _N + j] = o
  return eskf.EskfCovariance(v)


def _diag_r(r):
  return (r, 0.0, 0.0, r)


def _sample(e=3.0, n=4.0, r=0.01, valid=True):
  return spu.SurfacePositionSample(
      position_en_m=(e, n), R_en=_diag_r(r), valid=valid
  )


def _sample_r(r, e=3.0, n=4.0):
  return spu.SurfacePositionSample(
      position_en_m=(e, n), R_en=tuple(r), valid=True
  )


def _policy(is_surfaced=True, quality_ok=True):
  return spu.SurfaceFixPolicy(is_surfaced=is_surfaced, quality_ok=quality_ok)


def _hover_x(e=3.0, n=4.0):
  """Vehicle at rest at East 3, North 4, Up -10."""
  return eskf.EskfNominal(p_enu=(e, n, -10.0))


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


# nu = (0.1, -0.05), correlated R.
_GOLDEN_Z = _sample_r((0.02, 0.004, 0.004, 0.03), 1.1, 1.95)

# Independent numpy evaluation: K = sym(P) H^T S^-1, Joseph, right-error
# inject. Same constants as the C++ test.
_GOLDEN_D2 = 0.17928286852589673
_GOLDEN_NU = 0.11180339887498958
_GOLDEN_S = (0.07, 0.005, 0.005, 0.09)
_GOLDEN_NOMINAL = (
    1.0730677290836654,
    1.9632270916334662,
    -2.9991633466135457,
    0.9244238819514294,
    0.10352514147585345,
    -0.20434433105621927,
    0.3049039618067309,
    0.5040239043824701,
    -0.09916334661354582,
    0.2008366533864542,
    0.010836653386454185,
    -0.026533864541832677,
    0.03402390438247012,
    0.001836653386454184,
    0.002836653386454184,
    -0.002163346613545816,
)
_GOLDEN_P_DIAG = (
    0.01421195219123506,
    0.019921912350597606,
    0.06997609561752989,
    0.07997609561752989,
    0.08997609561752988,
    0.09975298804780877,
    0.109800796812749,
    0.1199760956175299,
    0.12997609561752987,
    0.13997609561752988,
    0.1497529880478088,
    0.159800796812749,
    0.16997609561752985,
    0.17997609561752986,
    0.18997609561752987,
)


def _update(x, p, z, policy=None, t=_CHI2):
  return spu.update_surface_position(
      x, p, z, _policy() if policy is None else policy, t
  )


class SurfacePositionUpdateTest(unittest.TestCase):

  def _assert_unchanged(self, r, status, evaluated=False):
    self.assertEqual(r.status, status)
    self.assertEqual(r.evaluated, evaluated)
    self.assertFalse(r.accepted)
    self.assertIsNone(r.nominal)
    self.assertIsNone(r.P)
    self.assertIsNone(r.delta_x)
    self.assertEqual(r.diagnostics is not None, evaluated)

  def test_surfaced_accept(self):
    # Identity hover at (3, 4) with a matching fix. S = 0.05 I2, K = 0.8 on
    # dp_e / dp_n, variance 0.04 -> 0.008.
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
          diag[i], 0.008 if i < 2 else 0.04, delta=_TOL, msg=str(i)
      )
      for j in range(_N):
        if i != j:
          self.assertEqual(r.P.at(i, j), 0.0)
    self.assertEqual(r.diagnostics.mahalanobis_sq, 0.0)
    self.assertEqual(r.diagnostics.innovation_norm, 0.0)
    self.assertEqual(r.diagnostics.threshold, _CHI2)
    self.assertEqual(r.diagnostics.dof, 2)
    self.assertEqual(len(r.diagnostics.S), 4)
    for got, want in zip(r.diagnostics.S, (0.05, 0.0, 0.0, 0.05)):
      self.assertAlmostEqual(got, want, delta=_TOL)

  def test_only_horizontal_position_variance_shrinks(self):
    r = _update(_hover_x(), _diag_p(0.04), _sample())
    self.assertLess(r.P.at(0, 0), 0.04)
    self.assertLess(r.P.at(1, 1), 0.04)
    for i in range(2, _N):
      self.assertEqual(r.P.at(i, i), 0.04)

  def test_offset_fix_moves_horizontal_position(self):
    # nu = (0.1, -0.05), d^2 = (0.01 + 0.0025) / 0.05 = 0.25,
    # dp = 0.8 * nu = (0.08, -0.04). Up is untouched.
    r = _update(_hover_x(), _diag_p(0.04), _sample(3.1, 3.95))
    self.assertEqual(r.status, _Status.OK_ACCEPT)
    self.assertAlmostEqual(r.diagnostics.mahalanobis_sq, 0.25, delta=_TOL)
    self.assertAlmostEqual(
        r.diagnostics.innovation_norm, math.sqrt(0.0125), delta=_TOL
    )
    self.assertAlmostEqual(r.nominal.p_enu[0], 3.08, delta=_TOL)
    self.assertAlmostEqual(r.nominal.p_enu[1], 3.96, delta=_TOL)
    self.assertEqual(r.nominal.p_enu[2], -10.0)
    dx = r.delta_x.to_tuple()
    self.assertAlmostEqual(dx[0], 0.08, delta=_TOL)
    self.assertAlmostEqual(dx[1], -0.04, delta=_TOL)
    self.assertEqual(dx[2], 0.0)
    self.assertEqual(r.nominal.q_wxyz, (1.0, 0.0, 0.0, 0.0))

  def test_east_and_north_are_not_swapped(self):
    r = _update(_hover_x(), _diag_p(0.04), _sample(3.1, 4.0))
    self.assertEqual(r.status, _Status.OK_ACCEPT)
    self.assertAlmostEqual(r.nominal.p_enu[0], 3.08, delta=_TOL)
    self.assertEqual(r.nominal.p_enu[1], 4.0)

  def test_correlated_state_is_injected(self):
    # nu = (0.1, 0), S = 0.05 I2, K[i, e] = P[i, e] / 0.05.
    p = _diag_p(
        0.04,
        {
            (0, 3): 0.02,
            (3, 0): 0.02,
            (9, 0): 0.005,
            (0, 9): 0.005,
            (13, 1): 0.0025,
            (1, 13): 0.0025,
        },
    )
    r = _update(_hover_x(), p, _sample(3.1, 4.0))
    self.assertEqual(r.status, _Status.OK_ACCEPT)
    n = r.nominal
    self.assertAlmostEqual(n.p_enu[0], 3.08, delta=_TOL)
    self.assertAlmostEqual(n.b_a[0], 0.01, delta=_TOL)
    self.assertAlmostEqual(n.b_g[0], 0.0, delta=_TOL)
    # dtheta_x = +0.04 -> q = normalize(1, 0.02, 0, 0).
    norm = math.sqrt(1.0 + 0.02 * 0.02)
    self.assertAlmostEqual(n.q_wxyz[0], 1.0 / norm, delta=_TOL)
    self.assertAlmostEqual(n.q_wxyz[1], 0.02 / norm, delta=_TOL)
    self.assertAlmostEqual(n.q_wxyz[2], 0.0, delta=_TOL)
    self.assertAlmostEqual(n.q_wxyz[3], 0.0, delta=_TOL)
    self.assertAlmostEqual(
        math.sqrt(sum(c * c for c in n.q_wxyz)), 1.0, delta=_TOL
    )
    self.assertAlmostEqual(r.delta_x.to_tuple()[3], 0.04, delta=_TOL)

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
    # nu = (1, 0), d^2 = 1 / 0.05 = 20.
    r = _update(x, p, _sample(4.0, 4.0))
    self._assert_unchanged(r, _Status.OK_REJECT, evaluated=True)
    self.assertAlmostEqual(r.diagnostics.mahalanobis_sq, 20.0, delta=_TOL)
    self.assertEqual(r.diagnostics.dof, 2)
    self.assertEqual(x.to_tuple(), x_before)
    self.assertEqual(p.row_major, p_before)

  def test_threshold_boundary(self):
    # nu = (0.1, 0), d^2 = 0.2.
    z = _sample(3.1, 4.0)
    self.assertEqual(
        _update(_hover_x(), _diag_p(0.04), z, None, 0.2 + 1e-6).status,
        _Status.OK_ACCEPT,
    )
    self.assertEqual(
        _update(_hover_x(), _diag_p(0.04), z, None, 0.2 - 1e-6).status,
        _Status.OK_REJECT,
    )

  def test_submerged_policy_skips_bit_identical(self):
    x = _golden_x()
    p = _golden_p()
    x_before = x.to_tuple()
    p_before = p.row_major
    self._assert_unchanged(
        _update(x, p, _GOLDEN_Z, _policy(is_surfaced=False)),
        _Status.SKIPPED_POLICY,
    )
    self.assertEqual(x.to_tuple(), x_before)
    self.assertEqual(p.row_major, p_before)

  def test_low_quality_policy_skips_bit_identical(self):
    x = _golden_x()
    p = _golden_p()
    x_before = x.to_tuple()
    p_before = p.row_major
    self._assert_unchanged(
        _update(x, p, _GOLDEN_Z, _policy(quality_ok=False)),
        _Status.SKIPPED_POLICY,
    )
    self.assertEqual(x.to_tuple(), x_before)
    self.assertEqual(p.row_major, p_before)

  def test_default_policy_and_both_flags_off_skip(self):
    self._assert_unchanged(
        _update(_hover_x(), _diag_p(0.04), _sample(), spu.SurfaceFixPolicy()),
        _Status.SKIPPED_POLICY,
    )
    self._assert_unchanged(
        _update(_hover_x(), _diag_p(0.04), _sample(), _policy(False, False)),
        _Status.SKIPPED_POLICY,
    )

  def test_policy_wins_over_everything_else(self):
    # Garbage sample, threshold, nominal, and P are not inspected when the
    # policy forbids the fix.
    bad_z = spu.SurfacePositionSample(
        position_en_m=(_NAN, _NAN), R_en=(_NAN,) * 4, valid=False
    )
    self._assert_unchanged(
        _update(
            eskf.EskfNominal(p_enu=(_NAN, 0.0, 0.0)),
            eskf.EskfCovariance(),
            bad_z,
            _policy(False, True),
            _NAN,
        ),
        _Status.SKIPPED_POLICY,
    )

  def test_invalid_flag(self):
    self._assert_unchanged(
        _update(_hover_x(), _diag_p(0.04), _sample(valid=False)),
        _Status.SKIPPED_INVALID,
    )

  def test_invalid_flag_wins_over_bad_contents(self):
    z = spu.SurfacePositionSample(
        position_en_m=(_NAN, _NAN), R_en=(_NAN, 0.0, 0.0, -1.0), valid=False
    )
    self._assert_unchanged(
        _update(_hover_x(), _diag_p(0.04), z), _Status.SKIPPED_INVALID
    )

  def test_non_finite_position_or_r(self):
    bad = [
        _sample(e=_NAN),
        _sample(n=_NAN),
        _sample(e=_INF),
        _sample(n=-_INF),
        _sample(r=_NAN),
        _sample(r=_INF),
        _sample_r((0.01, _NAN, 0.0, 0.01)),
    ]
    for z in bad:
      self._assert_unchanged(
          _update(_hover_x(), _diag_p(0.04), z), _Status.SKIPPED_INVALID
      )

  def test_wrong_shape_is_invalid(self):
    for z in (
        spu.SurfacePositionSample(
            position_en_m=(3.0,), R_en=_diag_r(0.01), valid=True
        ),
        spu.SurfacePositionSample(
            position_en_m=(3.0, 4.0), R_en=(0.01, 0.0, 0.01), valid=True
        ),
    ):
      self._assert_unchanged(
          _update(_hover_x(), _diag_p(0.04), z), _Status.SKIPPED_INVALID
      )

  def test_asymmetric_r_beyond_tolerance_is_invalid(self):
    self._assert_unchanged(
        _update(
            _hover_x(), _diag_p(0.04), _sample_r((0.01, 0.002, 0.001, 0.01))
        ),
        _Status.SKIPPED_INVALID,
    )
    self._assert_unchanged(
        _update(
            _hover_x(),
            _diag_p(0.04),
            _sample_r((0.01, 0.002, 0.002 + 2e-12, 0.01)),
        ),
        _Status.SKIPPED_INVALID,
    )

  def test_asymmetric_r_within_tolerance_evaluates(self):
    r = _update(
        _hover_x(),
        _diag_p(0.04),
        _sample_r((0.01, 0.002, 0.002 + 1e-13, 0.01)),
    )
    self.assertEqual(r.status, _Status.OK_ACCEPT)

  def test_non_positive_definite_r_is_singular(self):
    for r in (
        (0.0, 0.0, 0.0, 0.0),
        (-0.01, 0.0, 0.0, 0.01),
        (0.01, 0.0, 0.0, -0.01),
        (1e-12, 0.0, 0.0, 0.01),
        (0.01, 0.0, 0.0, 1e-13),
        (0.01, 0.02, 0.02, 0.01),
        (0.01, 0.01, 0.01, 0.01),
    ):
      self._assert_unchanged(
          _update(_hover_x(), _diag_p(0.04), _sample_r(r)), _Status.SINGULAR
      )

  def test_r_just_above_floor_evaluates(self):
    self.assertEqual(
        _update(_hover_x(), _diag_p(0.04), _sample(r=2e-12)).status,
        _Status.OK_ACCEPT,
    )

  def test_invalid_threshold(self):
    for t in (0.0, -1.0, _NAN, _INF):
      self._assert_unchanged(
          _update(_hover_x(), _diag_p(0.04), _sample(), None, t),
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
    # nu = (2, 0), S = 2 I2, d^2 = 2 passes the gate, but K[dp_z, e] = 1e200 / 2
    # and the Joseph product K * P[e, dp_z] overflows.
    p = _diag_p(1.0, {(0, 2): 1e200, (2, 0): 1e200})
    x = _hover_x()
    r = _update(x, p, _sample(5.0, 4.0, 1.0))
    self._assert_unchanged(r, _Status.NON_FINITE, evaluated=True)
    self.assertEqual(r.diagnostics.dof, 2)
    self.assertEqual(x.to_tuple(), _hover_x().to_tuple())

  def test_gate_overflow_is_non_finite(self):
    self._assert_unchanged(
        _update(
            _hover_x(),
            _diag_p(0.04, {(0, 0): 1.5e308}),
            _sample(r=1.5e308),
        ),
        _Status.NON_FINITE,
    )

  def test_innovation_overflow_is_non_finite(self):
    self._assert_unchanged(
        _update(_hover_x(1.7e308, 4.0), _diag_p(0.04), _sample(-1.7e308, 4.0)),
        _Status.NON_FINITE,
    )

  def test_golden_case(self):
    r = _update(_golden_x(), _golden_p(), _GOLDEN_Z)
    self.assertEqual(r.status, _Status.OK_ACCEPT)
    self.assertAlmostEqual(r.diagnostics.mahalanobis_sq, _GOLDEN_D2, delta=_TOL)
    self.assertAlmostEqual(
        r.diagnostics.innovation_norm, _GOLDEN_NU, delta=_TOL
    )
    self.assertEqual(r.diagnostics.dof, 2)
    for got, want in zip(r.diagnostics.S, _GOLDEN_S):
      self.assertAlmostEqual(got, want, delta=_TOL)
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
    policy = _policy()
    x_before = x.to_tuple()
    p_before = p.row_major
    z_before = dataclasses.astuple(z)
    policy_before = dataclasses.astuple(policy)
    _update(x, p, z, policy)
    self.assertEqual(x.to_tuple(), x_before)
    self.assertEqual(p.row_major, p_before)
    self.assertEqual(dataclasses.astuple(z), z_before)
    self.assertEqual(dataclasses.astuple(policy), policy_before)

  def test_determinism(self):
    a = _update(_golden_x(), _golden_p(), _GOLDEN_Z)
    b = _update(_golden_x(), _golden_p(), _GOLDEN_Z)
    self.assertEqual(a.nominal.to_tuple(), b.nominal.to_tuple())
    self.assertEqual(a.P.row_major, b.P.row_major)
    self.assertEqual(a.delta_x.to_tuple(), b.delta_x.to_tuple())
    self.assertEqual(a.diagnostics.mahalanobis_sq, b.diagnostics.mahalanobis_sq)
    self.assertEqual(a.diagnostics.S, b.diagnostics.S)

  def test_no_mode_transition_or_other_sensor_symbols(self):
    names = [n.lower() for n in dir(spu)]
    for cls in (
        spu.SurfacePositionSample,
        spu.SurfaceFixPolicy,
        spu.SurfacePositionUpdateResult,
    ):
      names += [f.name.lower() for f in dataclasses.fields(cls)]
    names += [s.name.lower() for s in _Status]
    for banned in (
        "mode",
        "transition",
        "depth",
        "altitude",
        "dvl",
        "baro",
        "water",
        "bottom",
    ):
      for n in names:
        self.assertNotIn(banned, n, msg=n)

  def test_status_set_is_closed(self):
    self.assertEqual(
        [s.name for s in _Status],
        [
            "OK_ACCEPT",
            "OK_REJECT",
            "SKIPPED_POLICY",
            "SKIPPED_INVALID",
            "SINGULAR",
            "NON_FINITE",
        ],
    )

  def test_sources_do_not_reference_other_updates_or_mode_writes(self):
    here = os.path.dirname(os.path.abspath(spu.__file__))
    for name in (
        "surface_position_update.h",
        "surface_position_update.cc",
        "surface_position_update.py",
    ):
      with open(os.path.join(here, name), encoding="utf-8") as f:
        text = f.read().lower()
      for banned in (
          "updatedepth",
          "update_depth",
          "depth_update",
          "updatealtitude",
          "update_altitude",
          "altitude_update",
          "updatedvl",
          "update_dvl",
          "dvl_",
          "navmode",
          "nav_mode",
          "transition",
      ):
        self.assertNotIn(banned, text, msg=f"{name}: {banned}")


if __name__ == "__main__":
  unittest.main()
