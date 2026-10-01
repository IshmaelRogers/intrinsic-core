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

"""Pure navigation-covariance rule over plain injected 3x3 matrices.

This does not read, run, or modify an estimator, ESKF, or filter, and does not
call a planner. A sample over a sigma limit is rejected, never projected.
"""

import dataclasses
import math
from typing import Sequence

from intrinsic.safety import safety_rule_result

NAV_COVARIANCE_RULE_ID = "nav.covariance"
DEFAULT_MAX_POSITION_SIGMA_M = 1.0
DEFAULT_MAX_VELOCITY_SIGMA_MPS = 0.5

_SYMMETRY_EPS = 1e-9
_MINOR_FLOOR = -1e-12

_BAD_CONFIG_SUMMARY = "nav covariance bad config"
_UNKNOWN_SUMMARY = "unknown covariance"
_POSITION_UNUSABLE_SUMMARY = "position covariance not usable"
_VELOCITY_UNUSABLE_SUMMARY = "velocity covariance not usable"

_RESULT = safety_rule_result


@dataclasses.dataclass(frozen=True)
class NavCovarianceSample:
  """Navigation covariance inputs supplied by the caller.

  `position_cov` and `velocity_cov` are row-major 3x3 matrices given as
  sequences of length 9. Units are m^2 for position and (m/s)^2 for velocity.
  `position_known` or `velocity_known` false means that covariance is unknown.
  """

  position_cov: Sequence[float]
  velocity_cov: Sequence[float]
  position_known: bool = True
  velocity_known: bool = True


def _critical(summary: str) -> _RESULT.SafetyRuleResult:
  return _RESULT.SafetyRuleResult(
      violated=True,
      rule_id=NAV_COVARIANCE_RULE_ID,
      severity=_RESULT.SEVERITY_CRITICAL,
      summary=summary,
      recommended_kind=_RESULT.DECISION_KIND_REJECT,
  )


def _is_valid_threshold(value: float) -> bool:
  return math.isfinite(value) and value > 0.0


def _is_usable(c: Sequence[float]) -> bool:
  """True if `c` is finite, symmetric, and PSD by leading principal minors."""
  try:
    if len(c) != 9:
      return False
    c = [float(x) for x in c]
  except (TypeError, ValueError):
    return False
  if not all(math.isfinite(x) for x in c):
    return False
  if (
      abs(c[1] - c[3]) > _SYMMETRY_EPS
      or abs(c[2] - c[6]) > _SYMMETRY_EPS
      or abs(c[5] - c[7]) > _SYMMETRY_EPS
  ):
    return False
  minor1 = c[0]
  minor2 = c[0] * c[4] - c[1] * c[3]
  minor3 = (
      c[0] * (c[4] * c[8] - c[5] * c[7])
      - c[1] * (c[3] * c[8] - c[5] * c[6])
      + c[2] * (c[3] * c[7] - c[4] * c[6])
  )
  return (
      minor1 >= _MINOR_FLOOR
      and minor2 >= _MINOR_FLOOR
      and minor3 >= _MINOR_FLOOR
  )


def _sigma(c: Sequence[float]) -> float:
  # Entries within the PSD floor may leave a tiny negative maximum diagonal.
  return math.sqrt(max(0.0, float(c[0]), float(c[4]), float(c[8])))


def _exceeded(metric: str, sigma: float) -> _RESULT.SafetyRuleResult:
  return _RESULT.SafetyRuleResult(
      violated=True,
      rule_id=NAV_COVARIANCE_RULE_ID,
      severity=_RESULT.SEVERITY_ERROR,
      summary=f"nav covariance exceeded metric={metric} sigma={sigma:.9g}",
      recommended_kind=_RESULT.DECISION_KIND_REJECT,
      has_projected_value=True,
      projected_value=sigma,
  )


def evaluate_nav_covariance_rule(
    sample: NavCovarianceSample,
    max_position_sigma_m: float = DEFAULT_MAX_POSITION_SIGMA_M,
    max_velocity_sigma_mps: float = DEFAULT_MAX_VELOCITY_SIGMA_MPS,
) -> _RESULT.SafetyRuleResult:
  """Evaluate `nav.covariance` for one sample.

  A matrix is usable when all nine entries are finite, `|C[i][j] - C[j][i]|
  <= 1e-9` for i < j, and it is positive semi-definite by its leading principal
  minors (C00, the leading 2x2 determinant, and det(C), each `>= -1e-12`). The
  sigma of a matrix is `sqrt(max(C00, C11, C22))`. Checked in this order:
    - Either threshold non-finite or `<= 0`: CRITICAL, REJECT.
    - `position_known` or `velocity_known` false: CRITICAL, REJECT. Fails
      closed even when the numbers would pass.
    - Unusable position matrix: CRITICAL, REJECT.
    - Unusable velocity matrix: CRITICAL, REJECT.
    - Position sigma `> max_position_sigma_m`: ERROR, REJECT with summary
      `nav covariance exceeded metric=position sigma=<value>`. Takes
      precedence over velocity when both are exceeded.
    - Else velocity sigma `> max_velocity_sigma_mps`: ERROR, REJECT with
      `metric=velocity`.
    - Otherwise (equal to a limit is compliant): compliant.
  On an exceeded result `has_projected_value` is true and `projected_value` is
  the observed sigma. This is for audit only. The decision kind is never
  PROJECT.
  """
  if not _is_valid_threshold(max_position_sigma_m) or not _is_valid_threshold(
      max_velocity_sigma_mps
  ):
    return _critical(_BAD_CONFIG_SUMMARY)
  if not sample.position_known or not sample.velocity_known:
    return _critical(_UNKNOWN_SUMMARY)
  if not _is_usable(sample.position_cov):
    return _critical(_POSITION_UNUSABLE_SUMMARY)
  if not _is_usable(sample.velocity_cov):
    return _critical(_VELOCITY_UNUSABLE_SUMMARY)
  position_sigma = _sigma(sample.position_cov)
  velocity_sigma = _sigma(sample.velocity_cov)
  if position_sigma > max_position_sigma_m:
    return _exceeded("position", position_sigma)
  if velocity_sigma > max_velocity_sigma_mps:
    return _exceeded("velocity", velocity_sigma)
  return _RESULT.SafetyRuleResult()
