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

"""Tests for the innovation gate. Mirrors innovation_gate_test.cc."""

import math
import unittest

from intrinsic.estimation import eskf_state as eskf
from intrinsic.estimation import innovation_gate as gate

_N = 15
_TOL = 1e-9
_NAN = float("nan")
_INF = float("inf")
_Status = gate.GateStatus


def _row_selector(rows):
  """H (len(rows) x 15) with a single 1 per row at the given column."""
  h = [0.0] * (len(rows) * _N)
  for i, col in enumerate(rows):
    h[i * _N + col] = 1.0
  return h


def _sparse_p(diag):
  """P with the given {index: value} diagonal entries, all else zero."""
  v = [0.0] * (_N * _N)
  for i, value in diag.items():
    v[i * _N + i] = value
  return eskf.EskfCovariance(v)


# Analytic: S = diag(3 + 1, 0.5 + 0.5) = diag(4, 1), nu = (2, 1) gives
# d^2 = 4/4 + 1/1 = 2 exactly in binary floating point.
def _boundary_inputs():
  nu = [2.0, 1.0]
  h = _row_selector([0, 1])
  p = _sparse_p({0: 3.0, 1: 0.5})
  r = [1.0, 0.0, 0.0, 0.5]
  return nu, h, p, r


def _golden_p():
  """Deliberately asymmetric, finite, deterministic."""
  return eskf.EskfCovariance(
      [
          1.0 + 0.1 * i if i == j else 0.01 * ((i * 7 + j * 3) % 5) - 0.02
          for i in range(_N)
          for j in range(_N)
      ]
  )


def _golden_inputs():
  nu = [0.3, -0.2, 0.5]
  h = [0.1 * ((i * 5 + k * 3) % 7) - 0.25 for i in range(3) for k in range(_N)]
  r = [0.5, 0.1, 0.0, 0.1, 0.4, 0.05, 0.0, 0.05, 0.3]
  return nu, h, _golden_p(), r


# Independent numpy evaluation of sym(S)^-1 with sym(P).
_GOLDEN_D2 = 0.3613017414836702
_GOLDEN_NORM = 0.6164414002968976
_GOLDEN_S = (
    (0, 0, 1.6041000000000003),
    (0, 1, -0.21315),
    (1, 2, -0.0508499999999999),
    (2, 2, 1.2932000000000003),
)


def _run(nu, h, p, r, threshold):
  return gate.gate_innovation(nu, h, p, r, threshold)


class InnovationGateTest(unittest.TestCase):

  def test_exact_boundary_accepts(self):
    nu, h, p, r = _boundary_inputs()
    res = _run(nu, h, p, r, 2.0)
    self.assertTrue(res.ok)
    self.assertTrue(res.evaluated)
    self.assertTrue(res.accepted)
    self.assertEqual(res.status, _Status.OK_ACCEPT)
    self.assertEqual(res.diagnostics.mahalanobis_sq, 2.0)

  def test_just_over_rejects(self):
    nu, h, p, r = _boundary_inputs()
    res = _run(nu, h, p, r, 2.0 - 1e-9)
    self.assertTrue(res.ok)
    self.assertTrue(res.evaluated)
    self.assertFalse(res.accepted)
    self.assertEqual(res.status, _Status.OK_REJECT)
    self.assertEqual(res.diagnostics.mahalanobis_sq, 2.0)
    res = _run([2.0 * (1.0 + 1e-6), 1.0], h, p, r, 2.0)
    self.assertEqual(res.status, _Status.OK_REJECT)
    self.assertGreater(res.diagnostics.mahalanobis_sq, 2.0)

  def test_deep_accept_and_reject(self):
    _, h, p, r = _boundary_inputs()
    res = _run([0.01, 0.01], h, p, r, 5.991)
    self.assertEqual(res.status, _Status.OK_ACCEPT)
    self.assertLess(res.diagnostics.mahalanobis_sq, 1e-3)
    res = _run([100.0, 100.0], h, p, r, 5.991)
    self.assertEqual(res.status, _Status.OK_REJECT)
    self.assertFalse(res.accepted)
    self.assertGreater(res.diagnostics.mahalanobis_sq, 1e3)

  def test_scalar_analytic(self):
    h = _row_selector([4])
    p = _sparse_p({4: 3.0})
    res = _run([2.0], h, p, [1.0], 1.0)
    self.assertEqual(res.status, _Status.OK_ACCEPT)
    self.assertEqual(res.diagnostics.mahalanobis_sq, 1.0)
    self.assertEqual(res.diagnostics.S, (4.0,))
    self.assertEqual(res.diagnostics.dof, 1)

  def test_singular_s(self):
    res = _run([1.0], [0.0] * _N, eskf.EskfCovariance.identity(), [0.0], 3.841)
    self.assertFalse(res.ok)
    self.assertFalse(res.evaluated)
    self.assertFalse(res.accepted)
    self.assertEqual(res.status, _Status.SINGULAR_S)
    self.assertIsNone(res.diagnostics)
    res = _run(
        [1.0, 1.0],
        [0.0] * (2 * _N),
        eskf.EskfCovariance.identity(),
        [0.0] * 4,
        5.991,
    )
    self.assertEqual(res.status, _Status.SINGULAR_S)

  def test_indefinite_s_is_singular(self):
    h = _row_selector([0])
    res = _run([1.0], h, eskf.EskfCovariance.identity(), [-2.0], 3.841)
    self.assertEqual(res.status, _Status.SINGULAR_S)
    self.assertIsNone(res.diagnostics)
    res = _run([1.0], h, eskf.EskfCovariance.identity(), [-1.0], 3.841)
    self.assertEqual(res.status, _Status.SINGULAR_S)

  def test_pivot_floor(self):
    res = _run([1.0], _row_selector([0]), _sparse_p({}), [1e-12], 3.841)
    self.assertEqual(res.status, _Status.SINGULAR_S)
    res = _run([1.0], _row_selector([0]), _sparse_p({}), [2e-12], 3.841)
    self.assertEqual(res.status, _Status.OK_REJECT)

  def test_non_finite_nu(self):
    nu, h, p, r = _boundary_inputs()
    for bad in (_NAN, _INF, -_INF):
      for i in range(2):
        v = list(nu)
        v[i] = bad
        res = _run(v, h, p, r, 2.0)
        self.assertFalse(res.ok)
        self.assertEqual(res.status, _Status.INVALID_NU)
        self.assertIsNone(res.diagnostics)

  def test_non_finite_p_and_unknown_p(self):
    nu, h, _, r = _boundary_inputs()
    res = _run(nu, h, eskf.EskfCovariance(), r, 2.0)
    self.assertEqual(res.status, _Status.INVALID_P)
    self.assertIsNone(res.diagnostics)
    for bad in (_NAN, _INF, -_INF):
      vals = [0.0] * (_N * _N)
      vals[17] = bad
      res = _run(nu, h, eskf.EskfCovariance(vals), r, 2.0)
      self.assertFalse(res.ok)
      self.assertEqual(res.status, _Status.INVALID_P)
      self.assertIsNone(res.diagnostics)

  def test_non_finite_r(self):
    nu, h, p, _ = _boundary_inputs()
    for bad in (_NAN, _INF, -_INF):
      for idx in (0, 1, 3):
        r = [1.0, 0.0, 0.0, 0.5]
        r[idx] = bad
        res = _run(nu, h, p, r, 2.0)
        self.assertFalse(res.ok)
        self.assertEqual(res.status, _Status.INVALID_R)
        self.assertIsNone(res.diagnostics)

  def test_invalid_threshold(self):
    nu, h, p, r = _boundary_inputs()
    for bad in (_NAN, _INF, -_INF, 0.0, -0.0, -1.0, -1e-300):
      res = _run(nu, h, p, r, bad)
      self.assertFalse(res.ok)
      self.assertEqual(res.status, _Status.INVALID_THRESHOLD)
      self.assertIsNone(res.diagnostics)
    self.assertTrue(_run(nu, h, p, r, 1e-300).ok)

  def test_non_finite_h(self):
    nu, h, p, r = _boundary_inputs()
    for bad in (_NAN, _INF):
      v = list(h)
      v[3] = bad
      res = _run(nu, v, p, r, 2.0)
      self.assertFalse(res.ok)
      self.assertEqual(res.status, _Status.NON_FINITE)
      self.assertIsNone(res.diagnostics)

  def test_overflow_is_non_finite(self):
    h = _row_selector([0])
    res = _run([1.0], h, _sparse_p({0: 1e308}), [1e308], 3.841)
    self.assertEqual(res.status, _Status.NON_FINITE)
    self.assertIsNone(res.diagnostics)
    res = _run([1e200], h, eskf.EskfCovariance.identity(), [1.0], 3.841)
    self.assertEqual(res.status, _Status.NON_FINITE)

  def test_dim_mismatch(self):
    nu, h, p, r = _boundary_inputs()
    cases = {
        "empty nu": ([], [], r),
        "h short": (nu, h[:-1], r),
        "h long": (nu, h + [0.0], r),
        "h 14 cols": (nu, [0.0] * (2 * 14), r),
        "h 16 cols": (nu, [0.0] * (2 * 16), r),
        "r short": (nu, h, r[:-1]),
        "r long": (nu, h, r + [0.0]),
        "r 1x1 for m=2": (nu, h, [1.0]),
    }
    for name, (n, hh, rr) in cases.items():
      res = _run(n, hh, p, rr, 2.0)
      self.assertFalse(res.ok, name)
      self.assertEqual(res.status, _Status.INVALID_DIM, name)
      self.assertIsNone(res.diagnostics, name)

  def test_asymmetric_r(self):
    nu, h, p, _ = _boundary_inputs()
    res = _run(nu, h, p, [1.0, 0.0, 1e-11, 0.5], 2.0)
    self.assertFalse(res.ok)
    self.assertEqual(res.status, _Status.INVALID_R)
    self.assertIsNone(res.diagnostics)
    res = _run(nu, h, p, [1.0, 0.25, -0.25, 0.5], 2.0)
    self.assertEqual(res.status, _Status.INVALID_R)
    res = _run(nu, h, p, [1.0, 0.0, 5e-13, 0.5], 2.0)
    self.assertTrue(res.ok)

  def test_reject_precedence(self):
    nu, h, p, r = _boundary_inputs()
    bad_p = eskf.EskfCovariance()
    bad_r = [_NAN, 0.0, 0.0, 0.5]
    bad_nu = [_NAN, 1.0]
    self.assertEqual(
        _run(nu, h[:-1], bad_p, bad_r, _NAN).status, _Status.INVALID_DIM
    )
    self.assertEqual(
        _run(bad_nu, h, bad_p, bad_r, _NAN).status, _Status.INVALID_P
    )
    self.assertEqual(_run(bad_nu, h, p, bad_r, _NAN).status, _Status.INVALID_R)
    self.assertEqual(_run(bad_nu, h, p, r, _NAN).status, _Status.INVALID_NU)
    self.assertEqual(_run(nu, h, p, r, _NAN).status, _Status.INVALID_THRESHOLD)

  def test_diagnostics_on_accept_and_reject(self):
    nu, h, p, r = _boundary_inputs()
    for threshold, status in (
        (2.0, _Status.OK_ACCEPT),
        (1.0, _Status.OK_REJECT),
    ):
      res = _run(nu, h, p, r, threshold)
      self.assertEqual(res.status, status)
      diag = res.diagnostics
      self.assertIsNotNone(diag)
      self.assertEqual(diag.threshold, threshold)
      self.assertEqual(diag.dof, 2)
      self.assertAlmostEqual(diag.innovation_norm, math.sqrt(5.0), delta=_TOL)
      self.assertEqual(diag.mahalanobis_sq, 2.0)
      self.assertEqual(diag.S, (4.0, 0.0, 0.0, 1.0))

  def test_p_is_symmetrized(self):
    nu, h, _, r = _boundary_inputs()
    base = [0.0] * (_N * _N)
    base[0] = 3.0
    base[_N + 1] = 0.5
    asym = list(base)
    asym[0 * _N + 1] = 0.4
    sym = list(base)
    sym[0 * _N + 1] = 0.2
    sym[1 * _N + 0] = 0.2
    a = _run(nu, h, eskf.EskfCovariance(asym), r, 2.0)
    b = _run(nu, h, eskf.EskfCovariance(sym), r, 2.0)
    self.assertEqual(a.diagnostics.mahalanobis_sq, b.diagnostics.mahalanobis_sq)
    self.assertEqual(a.diagnostics.S, b.diagnostics.S)
    self.assertEqual(a.diagnostics.S[1], a.diagnostics.S[2])

  def test_does_not_write_inputs(self):
    nu, h, p, r = _golden_inputs()
    nu0, h0, r0 = list(nu), list(h), list(r)
    p0 = p.row_major
    _run(nu, h, p, r, 7.815)
    self.assertEqual(nu, nu0)
    self.assertEqual(h, h0)
    self.assertEqual(r, r0)
    self.assertEqual(p.row_major, p0)

  def test_determinism_and_golden(self):
    args = (*_golden_inputs(), 7.815)
    first = _run(*args)
    second = _run(*args)
    self.assertEqual(first, second)
    self.assertEqual(first.status, _Status.OK_ACCEPT)
    diag = first.diagnostics
    self.assertAlmostEqual(diag.mahalanobis_sq, _GOLDEN_D2, delta=_TOL)
    self.assertAlmostEqual(diag.innovation_norm, _GOLDEN_NORM, delta=_TOL)
    self.assertEqual(diag.dof, 3)
    for row, col, value in _GOLDEN_S:
      self.assertAlmostEqual(diag.S[row * 3 + col], value, delta=_TOL)
      self.assertAlmostEqual(diag.S[col * 3 + row], value, delta=_TOL)

  def test_golden_threshold_straddle(self):
    args = _golden_inputs()
    self.assertEqual(_run(*args, _GOLDEN_D2 + 1e-6).status, _Status.OK_ACCEPT)
    self.assertEqual(_run(*args, _GOLDEN_D2 - 1e-6).status, _Status.OK_REJECT)


if __name__ == "__main__":
  unittest.main()
