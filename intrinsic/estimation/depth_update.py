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

"""Depth pressure / depth-sensor update only (#87).

Mirrors depth_update.h. See DEPTH_UPDATE.md. Depth is positive down from a
free surface given as an ENU Up coordinate. No altitude, barometric, or air
model, no DVL, lever arm, ICON, or hardware.
"""

import dataclasses
import enum
import math
from typing import Optional

from intrinsic.estimation import eskf_propagate
from intrinsic.estimation import eskf_state
from intrinsic.estimation import innovation_gate

_N = eskf_state.COV_DIM
_DP_Z = eskf_state.ERROR_DP + 2


@dataclasses.dataclass(frozen=True)
class DepthSample:
  """One depth sample as plain values (no marine proto)."""

  # Measured depth, positive down, SI meters. Must be finite.
  depth_m: float = 0.0
  # Scalar measurement variance, m^2. Must be finite and above
  # MIN_CHOLESKY_PIVOT.
  R: float = 0.0  # pylint: disable=invalid-name
  # Standalone stand-in for MeasurementHealth VALID. Must be true.
  valid: bool = False
  # ENU Up coordinate of the free surface, m. Must be finite.
  free_surface_up_m: float = 0.0


class DepthUpdateStatus(enum.Enum):
  OK_ACCEPT = 0
  OK_REJECT = 1
  SKIPPED_INVALID = 2
  SINGULAR = 3
  NON_FINITE = 4


@dataclasses.dataclass(frozen=True)
class DepthUpdateResult:
  # True when the gate ran to a decision (including a later NON_FINITE).
  evaluated: bool
  # True only for OK_ACCEPT.
  accepted: bool
  status: DepthUpdateStatus
  # #84 diagnostics (d^2, threshold, |nu|, dof = 1). Set iff evaluated.
  diagnostics: Optional[innovation_gate.GateDiagnostics] = None
  # Set only for OK_ACCEPT. Every other status leaves the caller's x and P
  # untouched, and these stay None.
  nominal: Optional[eskf_state.EskfNominal] = None
  P: Optional[eskf_state.EskfCovariance] = None  # pylint: disable=invalid-name
  # Error-state correction dx = K nu that was injected. Only for OK_ACCEPT.
  delta_x: Optional[eskf_state.EskfError] = None


def _all_finite(values) -> bool:
  return all(math.isfinite(x) for x in values)


def _skip(status: DepthUpdateStatus) -> DepthUpdateResult:
  return DepthUpdateResult(evaluated=False, accepted=False, status=status)


def _norm4(q) -> float:
  return math.sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3])


def update_depth(
    x: eskf_state.EskfNominal,
    p: eskf_state.EskfCovariance,
    z: DepthSample,
    chi2_threshold: float,
) -> DepthUpdateResult:
  """Depth update. Never modifies its inputs.

  h(x) = free_surface_up_m - p_enu[2], nu = z.depth_m - h(x), H = 1x15 with -1
  at the dp_z column and 0 elsewhere. Gates with gate_innovation (#84, dof =
  1), then applies a Joseph update and injects dx into the nominal (right
  attitude error). Computes on temporaries and commits only when everything is
  finite.

  Args:
    x: Prior nominal state (#81).
    p: Prior 15x15 covariance (#81).
    z: Depth sample.
    chi2_threshold: Finite and > 0, supplied by the caller (1 dof).

  Returns:
    A DepthUpdateResult. Checks run in this order and the first defect wins:
    valid, threshold, nominal and quaternion, P, depth / surface / R finite,
    R above the pivot floor, then the gate.
  """
  # 1. Health.
  if not z.valid:
    return _skip(DepthUpdateStatus.SKIPPED_INVALID)

  # 2. Threshold, nominal and quaternion, P, depth, surface, R.
  if not math.isfinite(chi2_threshold) or not chi2_threshold > 0.0:
    return _skip(DepthUpdateStatus.SKIPPED_INVALID)
  if not _all_finite(x.to_tuple()):
    return _skip(DepthUpdateStatus.SKIPPED_INVALID)
  q_norm_in = _norm4(x.q_wxyz)
  if (
      not math.isfinite(q_norm_in)
      or q_norm_in < eskf_propagate.MIN_QUATERNION_NORM
  ):
    return _skip(DepthUpdateStatus.SKIPPED_INVALID)
  if not p.has_value or not _all_finite(p.row_major):
    return _skip(DepthUpdateStatus.SKIPPED_INVALID)
  depth = float(z.depth_m)
  surface = float(z.free_surface_up_m)
  r = float(z.R)
  if not _all_finite((depth, surface, r)):
    return _skip(DepthUpdateStatus.SKIPPED_INVALID)

  # 3. R is 1x1, so its Cholesky pivot is R itself.
  if r <= innovation_gate.MIN_CHOLESKY_PIVOT:
    return _skip(DepthUpdateStatus.SINGULAR)

  # 4. Gate. h(x) = surface_up - p_z, so H has -1 at the dp_z column.
  h_x = surface - x.p_enu[2]
  nu = depth - h_x
  if not (math.isfinite(h_x) and math.isfinite(nu)):
    return _skip(DepthUpdateStatus.NON_FINITE)
  h = [0.0] * _N
  h[_DP_Z] = -1.0

  gate = innovation_gate.gate_innovation([nu], h, p, [r], chi2_threshold)
  if not gate.evaluated:
    if gate.status == innovation_gate.GateStatus.SINGULAR_S:
      return _skip(DepthUpdateStatus.SINGULAR)
    if gate.status == innovation_gate.GateStatus.NON_FINITE:
      return _skip(DepthUpdateStatus.NON_FINITE)
    return _skip(DepthUpdateStatus.SKIPPED_INVALID)

  if not gate.accepted:
    return DepthUpdateResult(
        evaluated=True,
        accepted=False,
        status=DepthUpdateStatus.OK_REJECT,
        diagnostics=gate.diagnostics,
    )

  def fail_non_finite() -> DepthUpdateResult:
    return DepthUpdateResult(
        evaluated=True,
        accepted=False,
        status=DepthUpdateStatus.NON_FINITE,
        diagnostics=gate.diagnostics,
    )

  # 5. Gain. K = P_s H^T / S_s, and H^T selects column dp_z with a sign flip.
  pm = p.row_major
  p_s = [
      0.5 * (pm[i * _N + j] + pm[j * _N + i])
      for i in range(_N)
      for j in range(_N)
  ]
  s = gate.diagnostics.S[0]
  if not math.isfinite(s) or s <= innovation_gate.MIN_CHOLESKY_PIVOT:
    return fail_non_finite()
  k = [-p_s[i * _N + _DP_Z] / s for i in range(_N)]

  # 6. dx = K nu.
  dx = [k[i] * nu for i in range(_N)]

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
  # (K H)[i][j] = -K[i] for j = dp_z and 0 otherwise.
  i_kh = [0.0] * (_N * _N)
  for i in range(_N):
    for j in range(_N):
      kh = -k[i] if j == _DP_Z else 0.0
      i_kh[i * _N + j] = (1.0 if i == j else 0.0) - kh
  a_mat = [0.0] * (_N * _N)  # (I - K H) P_s
  for i in range(_N):
    for j in range(_N):
      acc = 0.0
      for m in range(_N):
        acc += i_kh[i * _N + m] * p_s[m * _N + j]
      a_mat[i * _N + j] = acc
  p_tmp = [0.0] * (_N * _N)
  for i in range(_N):
    for j in range(_N):
      acc = 0.0
      for m in range(_N):
        acc += a_mat[i * _N + m] * i_kh[j * _N + m]
      acc += k[i] * r * k[j]
      p_tmp[i * _N + j] = acc
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

  return DepthUpdateResult(
      evaluated=True,
      accepted=True,
      status=DepthUpdateStatus.OK_ACCEPT,
      diagnostics=gate.diagnostics,
      nominal=x_new,
      P=eskf_state.EskfCovariance(p_out),
      delta_x=eskf_state.EskfError.from_sequence(dx),
  )
