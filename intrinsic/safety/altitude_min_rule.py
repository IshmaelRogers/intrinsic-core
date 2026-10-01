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

"""Pure minimum seafloor-altitude gate.

Altitude is meters positive up from the seafloor (clearance to the bottom).
This function does not read World, ICON, a HAL, or Gazebo and does not check
collisions, geofences, or horizontal clearance. It only clamps a scalar.
"""

import math

from intrinsic.safety import safety_rule_result

ALTITUDE_MIN_RULE_ID = "altitude.min"

# Fixture default in meters.
DEFAULT_MIN_ALTITUDE = 2.0

_UNKNOWN_SUMMARY = "altitude is unknown"
_UNUSABLE_SUMMARY = "altitude input is not usable"
_BELOW_SUMMARY = "altitude is below min_altitude"

_RESULT = safety_rule_result


def _reject_critical(summary: str) -> _RESULT.SafetyRuleResult:
  return _RESULT.SafetyRuleResult(
      violated=True,
      rule_id=ALTITUDE_MIN_RULE_ID,
      severity=_RESULT.SEVERITY_CRITICAL,
      summary=summary,
      recommended_kind=_RESULT.DECISION_KIND_REJECT,
  )


def evaluate_altitude_min_rule(
    altitude_known: bool,
    altitude: float,
    min_altitude: float = DEFAULT_MIN_ALTITUDE,
) -> _RESULT.SafetyRuleResult:
  """Evaluate `altitude.min`.

  Unknown altitude fails closed: CRITICAL recommending REJECT. Non-finite
  `altitude` or `min_altitude`, or a negative `min_altitude`, is also CRITICAL
  REJECT with no projection. `altitude >= min_altitude` is compliant. A lower
  altitude is ERROR recommending PROJECT with `projected_value` equal to
  `min_altitude`.
  """
  if not altitude_known:
    return _reject_critical(_UNKNOWN_SUMMARY)
  if (
      not math.isfinite(altitude)
      or not math.isfinite(min_altitude)
      or min_altitude < 0.0
  ):
    return _reject_critical(_UNUSABLE_SUMMARY)
  if altitude >= min_altitude:
    return _RESULT.SafetyRuleResult()
  return _RESULT.SafetyRuleResult(
      violated=True,
      rule_id=ALTITUDE_MIN_RULE_ID,
      severity=_RESULT.SEVERITY_ERROR,
      summary=_BELOW_SUMMARY,
      recommended_kind=_RESULT.DECISION_KIND_PROJECT,
      has_projected_value=True,
      projected_value=min_altitude,
  )
