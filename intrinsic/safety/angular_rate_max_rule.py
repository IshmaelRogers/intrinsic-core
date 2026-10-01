# Copyright 2026 Intrinsic Innovation LLC
#
# Licensed under the Apache License, Version 2.0(the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# https:  // www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Pure angular-rate-magnitude gate.

`angular_rate_mag` is the Euclidean magnitude of the body angular velocity,
radians/second, >= 0. This function does not read World, ICON, a HAL, or
Gazebo and does not build a DesiredMotion. It only clamps a scalar.
"""

import math

from intrinsic.safety import safety_rule_result

ANGULAR_RATE_MAX_RULE_ID = "rate.angular.max"

# Fixture default in radians/second.
DEFAULT_MAX_ANGULAR_RATE = 0.5

_UNUSABLE_SUMMARY = "angular rate input is not usable"
_EXCEEDED_SUMMARY = "angular rate exceeds max_angular_rate"

_RESULT = safety_rule_result


def evaluate_angular_rate_max_rule(
    angular_rate_mag: float, max_angular_rate: float = DEFAULT_MAX_ANGULAR_RATE
) -> _RESULT.SafetyRuleResult:
  """Evaluate `rate.angular.max`.

  Non-finite or negative `angular_rate_mag`, or a non-finite or negative
  `max_angular_rate`, is CRITICAL recommending REJECT with no projection.
  `angular_rate_mag <= max_angular_rate` is compliant. A greater value is ERROR
  recommending PROJECT with `projected_value` equal to `max_angular_rate`.
  """
  if (
      not math.isfinite(angular_rate_mag)
      or not math.isfinite(max_angular_rate)
      or angular_rate_mag < 0.0
      or max_angular_rate < 0.0
  ):
    return _RESULT.SafetyRuleResult(
        violated=True,
        rule_id=ANGULAR_RATE_MAX_RULE_ID,
        severity=_RESULT.SEVERITY_CRITICAL,
        summary=_UNUSABLE_SUMMARY,
        recommended_kind=_RESULT.DECISION_KIND_REJECT,
    )
  if angular_rate_mag <= max_angular_rate:
    return _RESULT.SafetyRuleResult()
  return _RESULT.SafetyRuleResult(
      violated=True,
      rule_id=ANGULAR_RATE_MAX_RULE_ID,
      severity=_RESULT.SEVERITY_ERROR,
      summary=_EXCEEDED_SUMMARY,
      recommended_kind=_RESULT.DECISION_KIND_PROJECT,
      has_projected_value=True,
      projected_value=max_angular_rate,
  )
