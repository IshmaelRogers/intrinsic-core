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

"""ESKF covariance transition and process noise only (#83).

Mirrors eskf_cov_propagate.h. See ESKF_COV_PROPAGATE.md. No measurement
update, gain, Joseph form, van Loan, or expm. Nominal propagation stays in
eskf_propagate.py (#82).
"""

import dataclasses
import enum
import math
from typing import Optional, Tuple

from intrinsic.estimation import eskf_propagate
from intrinsic.estimation import eskf_state

_N = eskf_state.COV_DIM
_Matrix = Tuple[float, ...]
_Mat3 = Tuple[float, ...]
_Vec3 = Tuple[float, float, float]

_IDENTITY3 = (1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0)

# Error-state block offsets (same as #81).
_DP = 0
_DTHETA = 3
_DV = 6
_DBA = 9
_DBG = 12


@dataclasses.dataclass(frozen=True)
class ProcessNoiseConfig:
  """Continuous noise sqrt-PSDs, isotropic per triad. All finite and >= 0."""

  # Accel white noise, (m/s^2)/sqrt(Hz).
  sigma_accel: float = 0.0
  # Gyro white noise, (rad/s)/sqrt(Hz).
  sigma_gyro: float = 0.0
  # Accel bias random walk, (m/s^2)/sqrt(s).
  sigma_accel_bias_rw: float = 0.0
  # Gyro bias random walk, (rad/s)/sqrt(s).
  sigma_gyro_bias_rw: float = 0.0


class CovPropagateStatus(enum.Enum):
  OK = 0
  INVALID_DT = 1
  INVALID_IMU = 2
  INVALID_QUATERNION = 3
  INVALID_P = 4
  INVALID_NOISE = 5
  NON_FINITE = 6


@dataclasses.dataclass(frozen=True)
class CovPropagateResult:
  ok: bool
  status: CovPropagateStatus
  # Set only when ok (15x15). A rejected call leaves no partial state.
  P: Optional[eskf_state.EskfCovariance] = None


def _all_finite(values) -> bool:
  return all(math.isfinite(x) for x in values)


def _reject(status: CovPropagateStatus) -> CovPropagateResult:
  return CovPropagateResult(ok=False, status=status, P=None)


def _skew(u: _Vec3) -> _Mat3:
  return (0.0, -u[2], u[1], u[2], 0.0, -u[0], -u[1], u[0], 0.0)


def _mul3(a: _Mat3, b: _Mat3) -> _Mat3:
  out = []
  for i in range(3):
    for j in range(3):
      s = 0.0
      for k in range(3):
        s += a[3 * i + k] * b[3 * k + j]
      out.append(s)
  return tuple(out)


def _add_block(f, block_row: int, block_col: int, m: _Mat3, scale: float):
  for i in range(3):
    for j in range(3):
      f[(block_row + i) * _N + (block_col + j)] += scale * m[3 * i + j]


def _check_inputs(x, imu, dt_s) -> Optional[CovPropagateStatus]:
  """Order mirrors #82: dt, IMU, nominal finite, quaternion norm."""
  if (
      not math.isfinite(dt_s)
      or not dt_s > 0.0
      or dt_s > eskf_propagate.MAX_PROPAGATE_DT_S
  ):
    return CovPropagateStatus.INVALID_DT
  if not (_all_finite(imu.accel_m_s2) and _all_finite(imu.gyro_rad_s)):
    return CovPropagateStatus.INVALID_IMU
  if not _all_finite(x.to_tuple()):
    return CovPropagateStatus.NON_FINITE
  q = x.q_wxyz
  q_norm = math.sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3])
  if not math.isfinite(q_norm) or q_norm < eskf_propagate.MIN_QUATERNION_NORM:
    return CovPropagateStatus.INVALID_QUATERNION
  return None


def _build_f(x, imu) -> list:
  omega = tuple(imu.gyro_rad_s[i] - x.b_g[i] for i in range(3))
  r = eskf_propagate.rotation_body_to_world(x.q_wxyz)
  g_b = eskf_propagate._mat_t_vec(r, eskf_propagate.GRAVITY_ENU)
  v_x = _skew(x.v_body)
  w_x = _skew(omega)
  g_x = _skew(g_b)

  f = [0.0] * (_N * _N)
  _add_block(f, _DP, _DTHETA, _mul3(r, v_x), -1.0)
  _add_block(f, _DP, _DV, r, 1.0)
  _add_block(f, _DTHETA, _DTHETA, w_x, -1.0)
  _add_block(f, _DTHETA, _DBG, _IDENTITY3, -1.0)
  _add_block(f, _DV, _DTHETA, g_x, 1.0)
  _add_block(f, _DV, _DV, w_x, -1.0)
  _add_block(f, _DV, _DBA, _IDENTITY3, -1.0)
  _add_block(f, _DV, _DBG, v_x, -1.0)
  return f


def _phi_from_f(f, dt_s: float) -> _Matrix:
  return tuple(
      (1.0 if i == j else 0.0) + f[i * _N + j] * dt_s
      for i in range(_N)
      for j in range(_N)
  )


def _valid_sigma(s: float) -> bool:
  return math.isfinite(s) and s >= 0.0


def _valid_noise(n: ProcessNoiseConfig) -> bool:
  return (
      _valid_sigma(n.sigma_accel)
      and _valid_sigma(n.sigma_gyro)
      and _valid_sigma(n.sigma_accel_bias_rw)
      and _valid_sigma(n.sigma_gyro_bias_rw)
  )


def _qd_from_noise(n: ProcessNoiseConfig, dt_s: float) -> list:
  """Qd = G Qc G^T dt. Each G block is identity, so the result is diagonal."""
  qd = [0.0] * (_N * _N)
  for first, sigma in (
      (_DTHETA, n.sigma_gyro),
      (_DV, n.sigma_accel),
      (_DBA, n.sigma_accel_bias_rw),
      (_DBG, n.sigma_gyro_bias_rw),
  ):
    for i in range(3):
      qd[(first + i) * _N + (first + i)] = sigma * sigma * dt_s
  return qd


def build_phi(x, imu, dt_s: float) -> Optional[_Matrix]:
  """Test helper. Phi = I + F dt, row-major 15x15, or None on a reject."""
  if _check_inputs(x, imu, dt_s) is not None:
    return None
  return _phi_from_f(_build_f(x, imu), dt_s)


def build_qd(noise: ProcessNoiseConfig, dt_s: float) -> Optional[_Matrix]:
  """Test helper. Qd = G Qc G^T dt, row-major 15x15, or None on a reject."""
  if (
      not math.isfinite(dt_s)
      or not dt_s > 0.0
      or dt_s > eskf_propagate.MAX_PROPAGATE_DT_S
      or not _valid_noise(noise)
  ):
    return None
  return tuple(_qd_from_noise(noise, dt_s))


def propagate_covariance(
    x: eskf_state.EskfNominal,
    imu: eskf_propagate.ImuSample,
    dt_s: float,
    P: eskf_state.EskfCovariance,
    noise: ProcessNoiseConfig,
) -> CovPropagateResult:
  """P_out = sym(Phi P Phi^T + Qd). Never raises on bad input."""
  status = _check_inputs(x, imu, dt_s)
  if status is not None:
    return _reject(status)
  if not P.has_value or not _all_finite(P.row_major):
    return _reject(CovPropagateStatus.INVALID_P)
  if not _valid_noise(noise):
    return _reject(CovPropagateStatus.INVALID_NOISE)

  phi = _phi_from_f(_build_f(x, imu), dt_s)
  qd = _qd_from_noise(noise, dt_s)
  p = P.row_major

  tmp = [0.0] * (_N * _N)
  for i in range(_N):
    for j in range(_N):
      s = 0.0
      for k in range(_N):
        s += phi[i * _N + k] * p[k * _N + j]
      tmp[i * _N + j] = s
  p_tmp = [0.0] * (_N * _N)
  for i in range(_N):
    for j in range(_N):
      s = 0.0
      for k in range(_N):
        s += tmp[i * _N + k] * phi[j * _N + k]
      p_tmp[i * _N + j] = s + qd[i * _N + j]
  out = tuple(
      0.5 * (p_tmp[i * _N + j] + p_tmp[j * _N + i])
      for i in range(_N)
      for j in range(_N)
  )
  if not _all_finite(out):
    return _reject(CovPropagateStatus.NON_FINITE)
  return CovPropagateResult(
      ok=True, status=CovPropagateStatus.OK, P=eskf_state.EskfCovariance(out)
  )
