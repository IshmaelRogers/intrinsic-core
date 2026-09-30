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

"""DVL bottom-track body-velocity update only (#85).

Mirrors dvl_bottom_track_update.h. See DVL_BOTTOM_TRACK_UPDATE.md. No
water-track, lever arm, Earth rate, other sensors, ICON, or hardware.
"""

import dataclasses
import enum
import math
from typing import Optional, Tuple

from intrinsic.estimation import eskf_propagate
from intrinsic.estimation import eskf_state
from intrinsic.estimation import innovation_gate

_N = eskf_state.COV_DIM
_M = 3


@dataclasses.dataclass(frozen=True)
class DvlBottomTrackSample:
  """One bottom-track velocity sample as plain values (no marine proto)."""

  # Bottom-track linear velocity in the body frame (REP-103), m/s.
  velocity_body_m_s: Tuple[float, float, float] = (0.0, 0.0, 0.0)
  # Row-major 3x3 measurement covariance, (m/s)^2. Must be finite, symmetric
  # within R_SYMMETRY_TOL, and Cholesky-capable with pivots above
  # MIN_CHOLESKY_PIVOT.
  R_body: Tuple[float, ...] = (0.0,) * 9  # pylint: disable=invalid-name
  # Must be true to attempt an update.
  bottom_lock: bool = False
  # Standalone stand-in for MeasurementHealth VALID. Must be true.
  valid: bool = False


class DvlUpdateStatus(enum.Enum):
  OK_ACCEPT = 0
  OK_REJECT = 1
  SKIPPED_LOCK_LOSS = 2
  SKIPPED_INVALID = 3
  SINGULAR = 4
  NON_FINITE = 5


@dataclasses.dataclass(frozen=True)
class DvlUpdateResult:
  # True when the gate ran to a decision (including a later NON_FINITE).
  evaluated: bool
  # True only for OK_ACCEPT.
  accepted: bool
  status: DvlUpdateStatus
  # #84 diagnostics (d^2, threshold, ||nu||, dof = 3). Set iff evaluated.
  diagnostics: Optional[innovation_gate.GateDiagnostics] = None
  # Set only for OK_ACCEPT. Every other status leaves the caller's x and P
  # untouched, and these stay None.
  nominal: Optional[eskf_state.EskfNominal] = None
  P: Optional[eskf_state.EskfCovariance] = None  # pylint: disable=invalid-name
  # Error-state correction dx = K nu that was injected. Only for OK_ACCEPT.
  delta_x: Optional[eskf_state.EskfError] = None


def _all_finite(values) -> bool:
  return all(math.isfinite(x) for x in values)


def _skip(status: DvlUpdateStatus) -> DvlUpdateResult:
  return DvlUpdateResult(evaluated=False, accepted=False, status=status)


def _norm4(q) -> float:
  return math.sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3])


def _cholesky3(a: list) -> bool:
  """In-place lower Cholesky of a row-major 3x3. False on a bad pivot."""
  for j in range(_M):
    pivot = a[j * _M + j]
    for k in range(j):
      pivot -= a[j * _M + k] * a[j * _M + k]
    if not math.isfinite(pivot) or pivot <= innovation_gate.MIN_CHOLESKY_PIVOT:
      return False
    diag = math.sqrt(pivot)
    a[j * _M + j] = diag
    for i in range(j + 1, _M):
      s = a[i * _M + j]
      for k in range(j):
        s -= a[i * _M + k] * a[j * _M + k]
      a[i * _M + j] = s / diag
  return True


def _cholesky_solve3(l: list, b: list) -> list:
  z = [0.0] * _M
  for i in range(_M):
    s = b[i]
    for k in range(i):
      s -= l[i * _M + k] * z[k]
    z[i] = s / l[i * _M + i]
  y = [0.0] * _M
  for i in range(_M - 1, -1, -1):
    s = z[i]
    for k in range(i + 1, _M):
      s -= l[k * _M + i] * y[k]
    y[i] = s / l[i * _M + i]
  return y


def update_dvl_bottom_track(
    x: eskf_state.EskfNominal,
    p: eskf_state.EskfCovariance,
    z: DvlBottomTrackSample,
    chi2_threshold: float,
) -> DvlUpdateResult:
  """DVL bottom-track body-velocity update. Never modifies its inputs.

  h(x) = v_body, nu = z.velocity_body_m_s - x.v_body, H = [0 | I3 at dv | 0].
  Gates with gate_innovation (#84), then applies a Joseph update and injects
  dx into the nominal (right attitude error). Computes on temporaries and
  commits only when everything is finite.

  Args:
    x: Prior nominal state (#81).
    p: Prior 15x15 covariance (#81).
    z: Bottom-track sample.
    chi2_threshold: Finite and > 0, supplied by the caller (3 dof).

  Returns:
    A DvlUpdateResult. Checks run in this order and the first defect wins: lock
    and valid, threshold, nominal and quaternion, P, velocity, R finite and
    symmetric, R Cholesky, then the gate.
  """
  # 1. Lock and health.
  if not z.bottom_lock or not z.valid:
    return _skip(DvlUpdateStatus.SKIPPED_LOCK_LOSS)

  # 2. Threshold, nominal and quaternion, P, velocity, R.
  if not math.isfinite(chi2_threshold) or not chi2_threshold > 0.0:
    return _skip(DvlUpdateStatus.SKIPPED_INVALID)
  if not _all_finite(x.to_tuple()):
    return _skip(DvlUpdateStatus.SKIPPED_INVALID)
  q_norm_in = _norm4(x.q_wxyz)
  if (
      not math.isfinite(q_norm_in)
      or q_norm_in < eskf_propagate.MIN_QUATERNION_NORM
  ):
    return _skip(DvlUpdateStatus.SKIPPED_INVALID)
  if not p.has_value or not _all_finite(p.row_major):
    return _skip(DvlUpdateStatus.SKIPPED_INVALID)
  if len(z.velocity_body_m_s) != _M or len(z.R_body) != _M * _M:
    return _skip(DvlUpdateStatus.SKIPPED_INVALID)
  if not _all_finite(z.velocity_body_m_s):
    return _skip(DvlUpdateStatus.SKIPPED_INVALID)
  r = [float(v) for v in z.R_body]
  if not _all_finite(r):
    return _skip(DvlUpdateStatus.SKIPPED_INVALID)
  for i in range(_M):
    for j in range(i + 1, _M):
      if abs(r[i * _M + j] - r[j * _M + i]) > innovation_gate.R_SYMMETRY_TOL:
        return _skip(DvlUpdateStatus.SKIPPED_INVALID)

  # 3. R must factor with pivots above the floor. Cholesky reads only the
  # lower triangle, and R is already symmetric within tolerance.
  r_l = list(r)
  if not _cholesky3(r_l):
    return _skip(DvlUpdateStatus.SINGULAR)

  # 4. Gate. h(x) = v_body, H selects the dv block.
  nu = [z.velocity_body_m_s[i] - x.v_body[i] for i in range(_M)]
  h = [0.0] * (_M * _N)
  for i in range(_M):
    h[i * _N + eskf_state.ERROR_DV + i] = 1.0

  gate = innovation_gate.gate_innovation(nu, h, p, r, chi2_threshold)
  if not gate.evaluated:
    if gate.status == innovation_gate.GateStatus.SINGULAR_S:
      return _skip(DvlUpdateStatus.SINGULAR)
    if gate.status == innovation_gate.GateStatus.NON_FINITE:
      return _skip(DvlUpdateStatus.NON_FINITE)
    return _skip(DvlUpdateStatus.SKIPPED_INVALID)

  if not gate.accepted:
    return DvlUpdateResult(
        evaluated=True,
        accepted=False,
        status=DvlUpdateStatus.OK_REJECT,
        diagnostics=gate.diagnostics,
    )

  def fail_non_finite() -> DvlUpdateResult:
    return DvlUpdateResult(
        evaluated=True,
        accepted=False,
        status=DvlUpdateStatus.NON_FINITE,
        diagnostics=gate.diagnostics,
    )

  # 5. Gain. Uses P_s and the gate's symmetrized S_s.
  pm = p.row_major
  p_s = [
      0.5 * (pm[i * _N + j] + pm[j * _N + i])
      for i in range(_N)
      for j in range(_N)
  ]
  s_l = list(gate.diagnostics.S)
  if not _cholesky3(s_l):
    return fail_non_finite()

  # K = P_s H^T S^-1. Row i of K solves S k_i = (P_s H^T)_i, S symmetric.
  k = [0.0] * (_N * _M)
  for i in range(_N):
    pht = [0.0] * _M
    for c in range(_M):
      s = 0.0
      for a in range(_N):
        s += p_s[i * _N + a] * h[c * _N + a]
      pht[c] = s
    ki = _cholesky_solve3(s_l, pht)
    for c in range(_M):
      k[i * _M + c] = ki[c]

  # 6. dx = K nu.
  dx = [0.0] * _N
  for i in range(_N):
    s = 0.0
    for c in range(_M):
      s += k[i * _M + c] * nu[c]
    dx[i] = s

  # 7. Inject into a nominal copy. Right attitude error, first-order Exp.
  def add(base, offset):
    return tuple(base[i] + dx[offset + i] for i in range(3))

  dq = (
      1.0,
      0.5 * dx[eskf_state.ERROR_DTHETA],
      0.5 * dx[eskf_state.ERROR_DTHETA + 1],
      0.5 * dx[eskf_state.ERROR_DTHETA + 2],
  )
  q_tmp = eskf_propagate.quaternion_multiply(tuple(x.q_wxyz), dq)
  q_norm = _norm4(q_tmp)
  if not math.isfinite(q_norm) or q_norm < eskf_propagate.MIN_QUATERNION_NORM:
    return fail_non_finite()
  x_new = eskf_state.EskfNominal(
      p_enu=add(x.p_enu, eskf_state.ERROR_DP),
      q_wxyz=tuple(c / q_norm for c in q_tmp),
      v_body=add(x.v_body, eskf_state.ERROR_DV),
      b_a=add(x.b_a, eskf_state.ERROR_DBA),
      b_g=add(x.b_g, eskf_state.ERROR_DBG),
  )

  # 8. Joseph: P = (I - K H) P_s (I - K H)^T + K R K^T, then symmetrize.
  i_kh = [0.0] * (_N * _N)
  for i in range(_N):
    for j in range(_N):
      kh = 0.0
      for c in range(_M):
        kh += k[i * _M + c] * h[c * _N + j]
      i_kh[i * _N + j] = (1.0 if i == j else 0.0) - kh
  a_mat = [0.0] * (_N * _N)  # (I - K H) P_s
  for i in range(_N):
    for j in range(_N):
      s = 0.0
      for m in range(_N):
        s += i_kh[i * _N + m] * p_s[m * _N + j]
      a_mat[i * _N + j] = s
  kr = [0.0] * (_N * _M)  # K R
  for i in range(_N):
    for c in range(_M):
      s = 0.0
      for m in range(_M):
        s += k[i * _M + m] * r[m * _M + c]
      kr[i * _M + c] = s
  p_tmp = [0.0] * (_N * _N)
  for i in range(_N):
    for j in range(_N):
      s = 0.0
      for m in range(_N):
        s += a_mat[i * _N + m] * i_kh[j * _N + m]
      for c in range(_M):
        s += kr[i * _M + c] * k[j * _M + c]
      p_tmp[i * _N + j] = s
  p_out = [
      0.5 * (p_tmp[i * _N + j] + p_tmp[j * _N + i])
      for i in range(_N)
      for j in range(_N)
  ]

  # 9. Commit only when everything is finite.
  if not (
      _all_finite(x_new.to_tuple()) and _all_finite(p_out) and _all_finite(dx)
  ):
    return fail_non_finite()

  return DvlUpdateResult(
      evaluated=True,
      accepted=True,
      status=DvlUpdateStatus.OK_ACCEPT,
      diagnostics=gate.diagnostics,
      nominal=x_new,
      P=eskf_state.EskfCovariance(p_out),
      delta_x=eskf_state.EskfError.from_sequence(dx),
  )
