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

"""Reusable Mahalanobis innovation gate only (#84).

Mirrors innovation_gate.h. See INNOVATION_GATE.md. No sensor residual, Kalman
gain, Joseph form, state or covariance write. The caller supplies the chi2
threshold.
"""

import dataclasses
import enum
import math
from typing import Optional, Sequence, Tuple

from intrinsic.estimation import eskf_state

_N = eskf_state.COV_DIM

# Absolute symmetry tolerance for R, and the Cholesky pivot floor for S.
R_SYMMETRY_TOL = 1e-12
MIN_CHOLESKY_PIVOT = 1e-12
# d^2 below this (more negative) is an unusable result, not a clamp.
MIN_MAHALANOBIS_SQ = -1e-9

kRSymmetryTol = R_SYMMETRY_TOL
kMinCholeskyPivot = MIN_CHOLESKY_PIVOT
kMinMahalanobisSq = MIN_MAHALANOBIS_SQ


class GateStatus(enum.Enum):
  OK_ACCEPT = 0
  OK_REJECT = 1
  INVALID_DIM = 2
  INVALID_P = 3
  INVALID_R = 4
  INVALID_NU = 5
  INVALID_THRESHOLD = 6
  SINGULAR_S = 7
  NON_FINITE = 8


@dataclasses.dataclass(frozen=True)
class GateDiagnostics:
  """Set only when the gate evaluated (OK_ACCEPT or OK_REJECT)."""

  # d^2 = nu^T S^-1 nu. A tiny negative value is stored as 0.0.
  mahalanobis_sq: float
  # Echo of the caller's chi2 threshold.
  threshold: float
  # ||nu||_2.
  innovation_norm: float
  # Degrees of freedom, m = len(nu).
  dof: int
  # Symmetrized innovation covariance S_s, row-major m x m.
  S: Tuple[float, ...]


@dataclasses.dataclass(frozen=True)
class GateResult:
  # True for OK_ACCEPT and OK_REJECT. Equal to `evaluated`.
  ok: bool
  # True when the gate ran to a decision (accept or reject).
  evaluated: bool
  # True only for OK_ACCEPT.
  accepted: bool
  status: GateStatus
  # None on every error status. No partial accept or reject.
  diagnostics: Optional[GateDiagnostics] = None


def _all_finite(values) -> bool:
  return all(math.isfinite(x) for x in values)


def _reject(status: GateStatus) -> GateResult:
  return GateResult(
      ok=False, evaluated=False, accepted=False, status=status, diagnostics=None
  )


def _is_symmetric(a: Sequence[float], m: int, tol: float) -> bool:
  for i in range(m):
    for j in range(i + 1, m):
      if abs(a[i * m + j] - a[j * m + i]) > tol:
        return False
  return True


def _cholesky(a: list, m: int) -> Optional[GateStatus]:
  """In-place lower Cholesky S = L L^T, row-major m x m.

  The pivot is the value under the square root.

  Returns:
    None on success, SINGULAR_S when any pivot is <= MIN_CHOLESKY_PIVOT, or
    NON_FINITE when a pivot is not finite.
  """
  for j in range(m):
    pivot = a[j * m + j]
    for k in range(j):
      pivot -= a[j * m + k] * a[j * m + k]
    if not math.isfinite(pivot):
      return GateStatus.NON_FINITE
    if pivot <= MIN_CHOLESKY_PIVOT:
      return GateStatus.SINGULAR_S
    diag = math.sqrt(pivot)
    a[j * m + j] = diag
    for i in range(j + 1, m):
      s = a[i * m + j]
      for k in range(j):
        s -= a[i * m + k] * a[j * m + k]
      a[i * m + j] = s / diag
  return None


def _cholesky_solve(l: Sequence[float], m: int, b: Sequence[float]) -> list:
  z = [0.0] * m
  for i in range(m):
    s = b[i]
    for k in range(i):
      s -= l[i * m + k] * z[k]
    z[i] = s / l[i * m + i]
  y = [0.0] * m
  for i in range(m - 1, -1, -1):
    s = z[i]
    for k in range(i + 1, m):
      s -= l[k * m + i] * y[k]
    y[i] = s / l[i * m + i]
  return y


def gate_innovation(
    nu: Sequence[float],
    h: Sequence[float],
    p: eskf_state.EskfCovariance,
    r: Sequence[float],
    chi2_threshold: float,
) -> GateResult:
  """Mahalanobis innovation gate. Never modifies its inputs.

  Args:
    nu: Innovation, length m >= 1.
    h: Jacobian, row-major m x COV_DIM.
    p: 15x15 prior covariance (#81).
    r: Measurement noise, row-major m x m.
    chi2_threshold: Finite and > 0. Supplied by the caller.

  Returns:
    A GateResult. Accepts iff d^2 <= chi2_threshold where
    S = H sym(P) H^T + R and d^2 = nu^T sym(S)^-1 nu by Cholesky. Checks run in
    the order dim, P, R, nu, threshold, H finite, and the first defect wins.
  """
  m = len(nu)
  if m < 1 or len(h) != m * _N or len(r) != m * m:
    return _reject(GateStatus.INVALID_DIM)

  if not p.has_value or not _all_finite(p.row_major):
    return _reject(GateStatus.INVALID_P)
  if not _all_finite(r) or not _is_symmetric(r, m, R_SYMMETRY_TOL):
    return _reject(GateStatus.INVALID_R)
  if not _all_finite(nu):
    return _reject(GateStatus.INVALID_NU)
  if not math.isfinite(chi2_threshold) or not chi2_threshold > 0.0:
    return _reject(GateStatus.INVALID_THRESHOLD)
  if not _all_finite(h):
    return _reject(GateStatus.NON_FINITE)

  pm = p.row_major
  p_s = [
      0.5 * (pm[i * _N + j] + pm[j * _N + i])
      for i in range(_N)
      for j in range(_N)
  ]

  hp = [0.0] * (m * _N)
  for i in range(m):
    for j in range(_N):
      s = 0.0
      for k in range(_N):
        s += h[i * _N + k] * p_s[k * _N + j]
      hp[i * _N + j] = s

  s_raw = [0.0] * (m * m)
  for i in range(m):
    for j in range(m):
      s = 0.0
      for k in range(_N):
        s += hp[i * _N + k] * h[j * _N + k]
      s_raw[i * m + j] = s + r[i * m + j]
  s_s = [
      0.5 * (s_raw[i * m + j] + s_raw[j * m + i])
      for i in range(m)
      for j in range(m)
  ]
  if not _all_finite(s_s):
    return _reject(GateStatus.NON_FINITE)

  l = list(s_s)
  failure = _cholesky(l, m)
  if failure is not None:
    return _reject(failure)

  y = _cholesky_solve(l, m, nu)
  d2 = 0.0
  for i in range(m):
    d2 += nu[i] * y[i]
  norm_sq = 0.0
  for i in range(m):
    norm_sq += nu[i] * nu[i]
  norm = math.sqrt(norm_sq)
  if (
      not math.isfinite(d2)
      or not math.isfinite(norm)
      or d2 < MIN_MAHALANOBIS_SQ
  ):
    return _reject(GateStatus.NON_FINITE)
  if d2 < 0.0:
    d2 = 0.0

  accepted = d2 <= chi2_threshold
  return GateResult(
      ok=True,
      evaluated=True,
      accepted=accepted,
      status=GateStatus.OK_ACCEPT if accepted else GateStatus.OK_REJECT,
      diagnostics=GateDiagnostics(
          mahalanobis_sq=d2,
          threshold=float(chi2_threshold),
          innovation_norm=norm,
          dof=m,
          S=tuple(s_s),
      ),
  )
