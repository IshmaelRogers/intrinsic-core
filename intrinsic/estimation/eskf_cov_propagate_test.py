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

"""Tests for ESKF covariance propagation. Mirrors eskf_cov_propagate_test.cc."""

import math
import unittest

from intrinsic.estimation import eskf_cov_propagate as cov
from intrinsic.estimation import eskf_propagate as prop
from intrinsic.estimation import eskf_state as eskf

_N = 15
_TOL = 1e-9
_NAN = float("nan")
_INF = float("inf")
_G = 9.80665
_Noise = cov.ProcessNoiseConfig
_Status = cov.CovPropagateStatus


def _hover_imu(q, omega=(0.0, 0.0, 0.0)):
  r = prop.rotation_body_to_world(q)
  g_body = prop._mat_t_vec(r, prop.GRAVITY_ENU)
  return prop.ImuSample(
      accel_m_s2=tuple(-c for c in g_body), gyro_rad_s=tuple(omega)
  )


def _golden_nominal():
  return eskf.EskfNominal(
      p_enu=(1.0, 2.0, 3.0),
      q_wxyz=(0.9, 0.1, -0.2, 0.3),
      v_body=(0.5, -0.25, 0.1),
      b_a=(0.01, -0.02, 0.03),
      b_g=(0.001, 0.002, -0.003),
  )


def _golden_imu():
  return prop.ImuSample(
      accel_m_s2=(0.3, -0.4, 9.5), gyro_rad_s=(0.05, -0.1, 0.2)
  )


def _golden_p():
  """Deliberately asymmetric, finite, deterministic."""
  return eskf.EskfCovariance(
      [
          (1.0 + 0.1 * i) if i == j else 0.01 * ((i * 7 + j * 3) % 5) - 0.02
          for i in range(_N)
          for j in range(_N)
      ]
  )


_GOLDEN_NOISE = _Noise(0.1, 0.01, 0.001, 0.0001)
_GOLDEN_DT = 0.01
# (row, col, value) from the Python implementation; pinned in the C++ test too.
_GOLDEN = [
    (0, 0, 1.000244208930194),
    (1, 1, 1.1001273573407204),
    (2, 2, 1.2002688521501386),
    (3, 3, 1.3001585455659999),
    (4, 4, 1.4001204463729997),
    (5, 5, 1.5001274996579999),
    (6, 6, 1.6119586155527448),
    (7, 7, 1.7127169341409083),
    (8, 8, 1.8024498947138858),
    (9, 9, 1.9000000099999999),
    (10, 10, 2.00000001),
    (11, 11, 2.10000001),
    (12, 12, 2.2000000001),
    (13, 13, 2.3000000001),
    (14, 14, 2.4000000001000004),
    (0, 1, 0.004770390235457063),
    (1, 5, 0.012895720342105264),
    (2, 9, 0.005146236842105263),
    (3, 13, -0.020034749999999997),
    (4, 2, -0.0006276264576315776),
    (5, 6, -0.004031543281503161),
    (6, 10, 0.005545342026315788),
    (7, 14, 0.016715360026315792),
    (8, 3, -0.012235875194768944),
    (9, 7, 0.004615360026315791),
    (10, 11, 0.004999999999999999),
    (11, 0, 0.004820763157894736),
    (12, 4, 0.004942300000000002),
    (13, 8, -0.03191070205263158),
    (14, 12, 0.005),
]


def _matmul(a, b):
  return [
      sum(a[i * _N + k] * b[k * _N + j] for k in range(_N))
      for i in range(_N)
      for j in range(_N)
  ]


def _transpose(a):
  return [a[j * _N + i] for i in range(_N) for j in range(_N)]


def _sym_err(m):
  return max(
      abs(m[i * _N + j] - m[j * _N + i]) for i in range(_N) for j in range(_N)
  )


def _min_eigenvalue(m):
  """Smallest eigenvalue of a symmetric matrix by cyclic Jacobi."""
  a = [list(m[i * _N : (i + 1) * _N]) for i in range(_N)]
  for _ in range(100):
    off = sum(a[i][j] ** 2 for i in range(_N) for j in range(_N) if i != j)
    if off < 1e-30:
      break
    for p in range(_N - 1):
      for q in range(p + 1, _N):
        if abs(a[p][q]) < 1e-300:
          continue
        theta = (a[q][q] - a[p][p]) / (2.0 * a[p][q])
        t = math.copysign(1.0, theta) / (abs(theta) + math.sqrt(theta**2 + 1.0))
        c = 1.0 / math.sqrt(t * t + 1.0)
        s = t * c
        for k in range(_N):
          akp, akq = a[k][p], a[k][q]
          a[k][p] = c * akp - s * akq
          a[k][q] = s * akp + c * akq
        for k in range(_N):
          apk, aqk = a[p][k], a[q][k]
          a[p][k] = c * apk - s * aqk
          a[q][k] = s * apk + c * aqk
  return min(a[i][i] for i in range(_N))


def _assert_near(test, actual, expected, tol=_TOL):
  test.assertEqual(len(actual), len(expected))
  for i, (a, e) in enumerate(zip(actual, expected)):
    test.assertAlmostEqual(a, e, delta=tol, msg=f"index {i}")


def _run(x, imu, dt, p, noise):
  return cov.propagate_covariance(x, imu, dt, p, noise)


class EskfCovPropagateTest(unittest.TestCase):

  def test_zero_noise_identity_p_is_phi_phi_t(self):
    x = _golden_nominal()
    imu = _golden_imu()
    phi = cov.build_phi(x, imu, 0.05)
    r = _run(x, imu, 0.05, eskf.EskfCovariance.identity(), _Noise())
    self.assertTrue(r.ok)
    self.assertEqual(r.status, _Status.OK)
    expected = _matmul(phi, _transpose(phi))
    sym = [
        0.5 * (expected[i * _N + j] + expected[j * _N + i])
        for i in range(_N)
        for j in range(_N)
    ]
    _assert_near(self, r.P.row_major, sym)

  def test_phi_blocks_follow_the_contract_table(self):
    x = _golden_nominal()
    imu = _golden_imu()
    dt = 0.02
    phi = cov.build_phi(x, imu, dt)
    r = prop.rotation_body_to_world(x.q_wxyz)
    omega = tuple(imu.gyro_rad_s[i] - x.b_g[i] for i in range(3))
    g_b = prop._mat_t_vec(r, prop.GRAVITY_ENU)

    def block(br, bc):
      return [
          phi[(3 * br + i) * _N + 3 * bc + j]
          for i in range(3)
          for j in range(3)
      ]

    def skew(u):
      return [0.0, -u[2], u[1], u[2], 0.0, -u[0], -u[1], u[0], 0.0]

    eye = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    zero = [0.0] * 9
    rv = cov._mul3(r, skew(x.v_body))
    w_x, g_x, v_x = skew(omega), skew(g_b), skew(x.v_body)
    expected = {
        (0, 0): eye,
        (0, 1): [-dt * c for c in rv],
        (0, 2): [dt * c for c in r],
        (0, 3): zero,
        (0, 4): zero,
        (1, 0): zero,
        (1, 1): [e - dt * c for e, c in zip(eye, w_x)],
        (1, 2): zero,
        (1, 3): zero,
        (1, 4): [-dt * c for c in eye],
        (2, 0): zero,
        (2, 1): [dt * c for c in g_x],
        (2, 2): [e - dt * c for e, c in zip(eye, w_x)],
        (2, 3): [-dt * c for c in eye],
        (2, 4): [-dt * c for c in v_x],
        (3, 0): zero,
        (3, 1): zero,
        (3, 2): zero,
        (3, 3): eye,
        (3, 4): zero,
        (4, 0): zero,
        (4, 1): zero,
        (4, 2): zero,
        (4, 3): zero,
        (4, 4): eye,
    }
    for (br, bc), e in expected.items():
      _assert_near(self, block(br, bc), e, 1e-15)

  def test_qd_diagonal(self):
    dt = 0.25
    qd = cov.build_qd(_Noise(2.0, 3.0, 5.0, 7.0), dt)
    expected = [0.0] * (_N * _N)
    for first, sigma in ((3, 3.0), (6, 2.0), (9, 5.0), (12, 7.0)):
      for i in range(3):
        expected[(first + i) * _N + first + i] = sigma * sigma * dt
    _assert_near(self, qd, expected, 1e-15)

  def test_zero_p_returns_qd(self):
    dt = 0.1
    noise = _Noise(0.2, 0.03, 0.004, 0.0005)
    r = _run(
        _golden_nominal(),
        _golden_imu(),
        dt,
        eskf.EskfCovariance([0.0] * 225),
        noise,
    )
    self.assertTrue(r.ok)
    _assert_near(self, r.P.row_major, cov.build_qd(noise, dt), 1e-15)

  def test_stationary_hover_analytic(self):
    pp, pth, pv, pba, pbg = 1.0, 2.0, 3.0, 4.0, 5.0
    dt = 0.1
    diag = [pp] * 3 + [pth] * 3 + [pv] * 3 + [pba] * 3 + [pbg] * 3
    p0 = eskf.EskfCovariance(
        [diag[i] if i == j else 0.0 for i in range(_N) for j in range(_N)]
    )
    x = eskf.EskfNominal()
    r = _run(x, _hover_imu(x.q_wxyz), dt, p0, _Noise())
    self.assertTrue(r.ok)

    # [g_b x] with g_b = (0, 0, -g) is [[0, g, 0], [-g, 0, 0], [0, 0, 0]].
    s = [0.0, _G, 0.0, -_G, 0.0, 0.0, 0.0, 0.0, 0.0]
    expected = [[0.0] * _N for _ in range(_N)]

    def add(row, col, value):
      expected[row][col] += value

    for i in range(3):
      add(i, i, pp + pv * dt * dt)  # pp
      add(i, 6 + i, dt * pv)  # pv
      add(6 + i, i, dt * pv)
      add(3 + i, 3 + i, pth + pbg * dt * dt)  # theta theta
      add(3 + i, 12 + i, -dt * pbg)  # theta bg
      add(12 + i, 3 + i, -dt * pbg)
      add(6 + i, 9 + i, -dt * pba)  # v ba
      add(9 + i, 6 + i, -dt * pba)
      add(9 + i, 9 + i, pba)
      add(12 + i, 12 + i, pbg)
      # vv: pv I + dt^2 pth S S^T + dt^2 pba I, with S S^T = diag(g^2, g^2, 0).
      add(6 + i, 6 + i, pv + dt * dt * pba)
    add(6, 6, dt * dt * pth * _G * _G)
    add(7, 7, dt * dt * pth * _G * _G)
    for i in range(3):
      for j in range(3):
        add(6 + i, 3 + j, dt * pth * s[3 * i + j])  # v theta
        add(3 + j, 6 + i, dt * pth * s[3 * i + j])
    flat = [expected[i][j] for i in range(_N) for j in range(_N)]
    _assert_near(self, r.P.row_major, flat)
    # Pinned numbers: 1 + 3 dt^2, 3 + dt^2 (2 g^2 + 4), +/- dt*2*g, -dt*5.
    self.assertAlmostEqual(r.P.at(0, 0), 1.03, delta=_TOL)
    self.assertAlmostEqual(
        r.P.at(6, 6), 3.0 + 0.01 * (2.0 * _G * _G + 4.0), delta=_TOL
    )
    self.assertAlmostEqual(r.P.at(6, 4), 0.1 * 2.0 * _G, delta=_TOL)
    self.assertAlmostEqual(r.P.at(7, 3), -0.1 * 2.0 * _G, delta=_TOL)
    self.assertAlmostEqual(r.P.at(3, 12), -0.5, delta=_TOL)

  def test_symmetry_for_asymmetric_p(self):
    seed = 12345
    vals = []
    for _ in range(_N * _N):
      seed = (1103515245 * seed + 12345) % (2**31)
      vals.append(seed / 2**31 - 0.5)
    p = eskf.EskfCovariance(vals)
    self.assertGreater(
        max(
            abs(vals[i * _N + j] - vals[j * _N + i])
            for i in range(_N)
            for j in range(_N)
        ),
        0.1,
    )
    r = _run(_golden_nominal(), _golden_imu(), 0.05, p, _GOLDEN_NOISE)
    self.assertTrue(r.ok)
    self.assertLessEqual(_sym_err(r.P.row_major), 1e-12)

  def test_psd_floor(self):
    x = eskf.EskfNominal()
    eps = 1e-6
    p = eskf.EskfCovariance(
        [eps if i == j else 0.0 for i in range(_N) for j in range(_N)]
    )
    r = _run(x, _hover_imu(x.q_wxyz), 0.1, p, _Noise())
    self.assertTrue(r.ok)
    self.assertGreaterEqual(_min_eigenvalue(r.P.row_major), -1e-9)

    r = _run(
        _golden_nominal(),
        _golden_imu(),
        _GOLDEN_DT,
        eskf.EskfCovariance.identity(),
        _GOLDEN_NOISE,
    )
    self.assertTrue(r.ok)
    self.assertGreaterEqual(_min_eigenvalue(r.P.row_major), -1e-9)

  def test_dt_rejects(self):
    x, imu, p = _golden_nominal(), _golden_imu(), eskf.EskfCovariance.identity()
    for dt in (0.0, -0.01, _NAN, _INF, -_INF, 1.0 + 1e-9):
      r = _run(x, imu, dt, p, _Noise())
      self.assertFalse(r.ok)
      self.assertEqual(r.status, _Status.INVALID_DT)
      self.assertIsNone(r.P)
    self.assertTrue(_run(x, imu, 1.0, p, _Noise()).ok)

  def test_imu_rejects(self):
    x, p = _golden_nominal(), eskf.EskfCovariance.identity()
    for bad in (_NAN, _INF):
      for i in range(3):
        a = [0.0, 0.0, 0.0]
        a[i] = bad
        for imu in (
            prop.ImuSample(accel_m_s2=tuple(a)),
            prop.ImuSample(gyro_rad_s=tuple(a)),
        ):
          r = _run(x, imu, 0.01, p, _Noise())
          self.assertFalse(r.ok)
          self.assertEqual(r.status, _Status.INVALID_IMU)
          self.assertIsNone(r.P)

  def test_quaternion_and_nominal_rejects(self):
    imu, p = _golden_imu(), eskf.EskfCovariance.identity()
    r = _run(
        eskf.EskfNominal(q_wxyz=(0.0, 0.0, 0.0, 0.0)), imu, 0.01, p, _Noise()
    )
    self.assertEqual(r.status, _Status.INVALID_QUATERNION)
    self.assertFalse(r.ok)
    r = _run(
        eskf.EskfNominal(q_wxyz=(1e-13, 0.0, 0.0, 0.0)), imu, 0.01, p, _Noise()
    )
    self.assertEqual(r.status, _Status.INVALID_QUATERNION)
    r = _run(eskf.EskfNominal(v_body=(_NAN, 0.0, 0.0)), imu, 0.01, p, _Noise())
    self.assertFalse(r.ok)
    self.assertEqual(r.status, _Status.NON_FINITE)
    self.assertIsNone(r.P)

  def test_p_rejects(self):
    x, imu = _golden_nominal(), _golden_imu()
    r = _run(x, imu, 0.01, eskf.EskfCovariance(), _Noise())
    self.assertFalse(r.ok)
    self.assertEqual(r.status, _Status.INVALID_P)
    self.assertIsNone(r.P)
    for bad in (_NAN, _INF):
      vals = [0.0] * 225
      vals[17] = bad
      r = _run(x, imu, 0.01, eskf.EskfCovariance(vals), _Noise())
      self.assertFalse(r.ok)
      self.assertEqual(r.status, _Status.INVALID_P)
      self.assertIsNone(r.P)

  def test_noise_rejects(self):
    x, imu, p = _golden_nominal(), _golden_imu(), eskf.EskfCovariance.identity()
    for bad in (-1e-12, _NAN, _INF, -_INF):
      for field in (
          "sigma_accel",
          "sigma_gyro",
          "sigma_accel_bias_rw",
          "sigma_gyro_bias_rw",
      ):
        r = _run(x, imu, 0.01, p, _Noise(**{field: bad}))
        self.assertFalse(r.ok)
        self.assertEqual(r.status, _Status.INVALID_NOISE)
        self.assertIsNone(r.P)
    self.assertIsNone(cov.build_qd(_Noise(sigma_gyro=-1.0), 0.01))

  def test_reject_precedence(self):
    bad_noise = _Noise(sigma_accel=-1.0)
    bad_p = eskf.EskfCovariance()
    bad_imu = prop.ImuSample(accel_m_s2=(_NAN, 0.0, 0.0))
    r = _run(_golden_nominal(), bad_imu, 0.0, bad_p, bad_noise)
    self.assertEqual(r.status, _Status.INVALID_DT)
    r = _run(_golden_nominal(), bad_imu, 0.01, bad_p, bad_noise)
    self.assertEqual(r.status, _Status.INVALID_IMU)
    r = _run(_golden_nominal(), _golden_imu(), 0.01, bad_p, bad_noise)
    self.assertEqual(r.status, _Status.INVALID_P)

  def test_overflowing_output_is_non_finite(self):
    x = eskf.EskfNominal(v_body=(1e200, 0.0, 0.0))
    r = _run(x, _golden_imu(), 0.5, eskf.EskfCovariance.identity(), _Noise())
    self.assertFalse(r.ok)
    self.assertEqual(r.status, _Status.NON_FINITE)
    self.assertIsNone(r.P)

  def test_determinism_and_golden(self):
    args = (
        _golden_nominal(),
        _golden_imu(),
        _GOLDEN_DT,
        _golden_p(),
        _GOLDEN_NOISE,
    )
    first = _run(*args)
    second = _run(*args)
    self.assertTrue(first.ok)
    self.assertEqual(first.P.row_major, second.P.row_major)
    for row, col, value in _GOLDEN:
      self.assertAlmostEqual(first.P.at(row, col), value, delta=_TOL)


if __name__ == "__main__":
  unittest.main()
