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

"""Terrain / seafloor altimeter clearance update only (#88).

Mirrors altitude_update.h. See ALTITUDE_UPDATE.md. Altitude is the clearance
above the seafloor, positive up, and the seafloor is an external ENU Up
coordinate. Not barometric ASL, no free-surface model, no DVL, lever arm, ICON,
or hardware.
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
class SeafloorContext:
  """External seafloor under the vehicle. Plain value, supplied by the caller."""

  # False means no seafloor is known and the update is skipped.
  present: bool = False
  # ENU Up coordinate of the seafloor under the vehicle, m. Finite when
  # present.
  seafloor_up_m: float = 0.0


@dataclasses.dataclass(frozen=True)
class AltitudeSample:
  """One altimeter sample as plain values (no marine proto)."""

  # Measured clearance above the seafloor, positive up, SI meters. Must be
  # finite.
  altitude_m: float = 0.0
  # Scalar measurement variance, m^2. Must be finite and above
  # MIN_CHOLESKY_PIVOT.
  R: float = 0.0  # pylint: disable=invalid-name
  # Standalone stand-in for MeasurementHealth VALID. Must be true.
  valid: bool = False


class AltitudeUpdateStatus(enum.Enum):
  OK_ACCEPT = 0
  OK_REJECT = 1
  SKIPPED_MISSING_SEAFLOOR = 2
  SKIPPED_INVALID = 3
  SINGULAR = 4
  NON_FINITE = 5


@dataclasses.dataclass(frozen=True)
class AltitudeUpdateResult:
  # True when the gate ran to a decision (including a later NON_FINITE).
  evaluated: bool
  # True only for OK_ACCEPT.
  accepted: bool
  status: AltitudeUpdateStatus
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


def _skip(status: AltitudeUpdateStatus) -> AltitudeUpdateResult:
  return AltitudeUpdateResult(evaluated=False, accepted=False, status=status)


def _norm4(q) -> float:
  return math.sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3])


def update_altitude(
    x: eskf_state.EskfNominal,
    p: eskf_state.EskfCovariance,
    z: AltitudeSample,
    seafloor: SeafloorContext,
    chi2_threshold: float,
) -> AltitudeUpdateResult:
  """Altitude update. Never modifies its inputs.

  h(x) = p_enu[2] - seafloor_up_m, nu = z.altitude_m - h(x), H = 1x15 with +1
  at the dp_z column and 0 elsewhere. Gates with gate_innovation (#84, dof =
  1), then applies a Joseph update and injects dx into the nominal (right
  attitude error). Computes on temporaries and commits only when everything is
  finite.

  Args:
    x: Prior nominal state (#81).
    p: Prior 15x15 covariance (#81).
    z: Altimeter sample.
    seafloor: External seafloor context. Must be present and finite.
    chi2_threshold: Finite and > 0, supplied by the caller (1 dof).

  Returns:
    An AltitudeUpdateResult. Checks run in this order and the first defect wins:
    valid, threshold, nominal and quaternion, P, seafloor present and finite,
    altitude / R finite, R above the pivot floor, then the gate.
  """
  # 1. Health.
  if not z.valid:
    return _skip(AltitudeUpdateStatus.SKIPPED_INVALID)

  # 2. Threshold, nominal and quaternion, P.
  if not math.isfinite(chi2_threshold) or not chi2_threshold > 0.0:
    return _skip(AltitudeUpdateStatus.SKIPPED_INVALID)
  if not _all_finite(x.to_tuple()):
    return _skip(AltitudeUpdateStatus.SKIPPED_INVALID)
  q_norm_in = _norm4(x.q_wxyz)
  if (
      not math.isfinite(q_norm_in)
      or q_norm_in < eskf_propagate.MIN_QUATERNION_NORM
  ):
    return _skip(AltitudeUpdateStatus.SKIPPED_INVALID)
  if not p.has_value or not _all_finite(p.row_major):
    return _skip(AltitudeUpdateStatus.SKIPPED_INVALID)

  # 3. Seafloor context, then altitude and R.
  if not seafloor.present or not math.isfinite(seafloor.seafloor_up_m):
    return _skip(AltitudeUpdateStatus.SKIPPED_MISSING_SEAFLOOR)
  altitude = float(z.altitude_m)
  seafloor_up = float(seafloor.seafloor_up_m)
  r = float(z.R)
  if not _all_finite((altitude, r)):
    return _skip(AltitudeUpdateStatus.SKIPPED_INVALID)

  # 4. R is 1x1, so its Cholesky pivot is R itself.
  if r <= innovation_gate.MIN_CHOLESKY_PIVOT:
    return _skip(AltitudeUpdateStatus.SINGULAR)

  # 5. Gate. h(x) = p_z - seafloor_up, so H has +1 at the dp_z column.
  h_x = x.p_enu[2] - seafloor_up
  nu = altitude - h_x
  if not (math.isfinite(h_x) and math.isfinite(nu)):
    return _skip(AltitudeUpdateStatus.NON_FINITE)
  h = [0.0] * _N
  h[_DP_Z] = 1.0

  gate = innovation_gate.gate_innovation([nu], h, p, [r], chi2_threshold)
  if not gate.evaluated:
    if gate.status == innovation_gate.GateStatus.SINGULAR_S:
      return _skip(AltitudeUpdateStatus.SINGULAR)
    if gate.status == innovation_gate.GateStatus.NON_FINITE:
      return _skip(AltitudeUpdateStatus.NON_FINITE)
    return _skip(AltitudeUpdateStatus.SKIPPED_INVALID)

  if not gate.accepted:
    return AltitudeUpdateResult(
        evaluated=True,
        accepted=False,
        status=AltitudeUpdateStatus.OK_REJECT,
        diagnostics=gate.diagnostics,
    )

  def fail_non_finite() -> AltitudeUpdateResult:
    return AltitudeUpdateResult(
        evaluated=True,
        accepted=False,
        status=AltitudeUpdateStatus.NON_FINITE,
        diagnostics=gate.diagnostics,
    )

  # 6. Gain. K = P_s H^T / S_s, and H^T selects column dp_z.
  pm = p.row_major
  p_s = [
      0.5 * (pm[i * _N + j] + pm[j * _N + i])
      for i in range(_N)
      for j in range(_N)
  ]
  s = gate.diagnostics.S[0]
  if not math.isfinite(s) or s <= innovation_gate.MIN_CHOLESKY_PIVOT:
    return fail_non_finite()
  k = [p_s[i * _N + _DP_Z] / s for i in range(_N)]

  # 7. dx = K nu.
  dx = [k[i] * nu for i in range(_N)]

  # 8. Inject into a nominal copy. Right attitude error, first-order Exp.
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

  # 9. Joseph: P = (I - K H) P_s (I - K H)^T + K R K^T, then symmetrize.
  # (K H)[i][j] = K[i] for j = dp_z and 0 otherwise.
  i_kh = [0.0] * (_N * _N)
  for i in range(_N):
    for j in range(_N):
      kh = k[i] if j == _DP_Z else 0.0
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

  # 10. Commit only when everything is finite.
  if not (
      _all_finite(x_new.to_tuple()) and _all_finite(p_out) and _all_finite(dx)
  ):
    return fail_non_finite()

  return AltitudeUpdateResult(
      evaluated=True,
      accepted=True,
      status=AltitudeUpdateStatus.OK_ACCEPT,
      diagnostics=gate.diagnostics,
      nominal=x_new,
      P=eskf_state.EskfCovariance(p_out),
      delta_x=eskf_state.EskfError.from_sequence(dx),
  )
