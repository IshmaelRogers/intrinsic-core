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

"""Pure linear-speed gate.

`speed` is the Euclidean magnitude of the body linear velocity,
meters/second, >= 0. This function does not read World, ICON, a HAL, or
Gazebo and does not build a DesiredMotion. It only clamps a scalar.
"""

import math

from intrinsic.safety import safety_rule_result

SPEED_MAX_RULE_ID = "speed.linear.max"

# Fixture default in meters/second.
DEFAULT_MAX_LINEAR_SPEED = 1.5

_UNUSABLE_SUMMARY = "linear speed input is not usable"
_EXCEEDED_SUMMARY = "linear speed exceeds max_linear_speed"

_RESULT = safety_rule_result


def evaluate_speed_max_rule(
    speed: float, max_linear_speed: float = DEFAULT_MAX_LINEAR_SPEED
) -> _RESULT.SafetyRuleResult:
  """Evaluate `speed.linear.max`.

  Non-finite or negative `speed`, or a non-finite or negative
  `max_linear_speed`, is CRITICAL recommending REJECT with no projection.
  `speed <= max_linear_speed` is compliant. A greater value is ERROR
  recommending PROJECT with `projected_value` equal to `max_linear_speed`.
  """
  if (
      not math.isfinite(speed)
      or not math.isfinite(max_linear_speed)
      or speed < 0.0
      or max_linear_speed < 0.0
  ):
    return _RESULT.SafetyRuleResult(
        violated=True,
        rule_id=SPEED_MAX_RULE_ID,
        severity=_RESULT.SEVERITY_CRITICAL,
        summary=_UNUSABLE_SUMMARY,
        recommended_kind=_RESULT.DECISION_KIND_REJECT,
    )
  if speed <= max_linear_speed:
    return _RESULT.SafetyRuleResult()
  return _RESULT.SafetyRuleResult(
      violated=True,
      rule_id=SPEED_MAX_RULE_ID,
      severity=_RESULT.SEVERITY_ERROR,
      summary=_EXCEEDED_SUMMARY,
      recommended_kind=_RESULT.DECISION_KIND_PROJECT,
      has_projected_value=True,
      projected_value=max_linear_speed,
  )
