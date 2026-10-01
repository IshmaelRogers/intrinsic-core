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

"""Pure heave (depth-rate) gate.

`depth_rate` is meters/second with depth positive down, so a positive rate is
descent and a negative rate is ascent. This function does not read World, ICON,
a HAL, or Gazebo and does not build a DesiredMotion. It only clamps a scalar.
"""

import math

from intrinsic.safety import safety_rule_result

DESCENT_RATE_MAX_RULE_ID = "rate.descent.max"
ASCENT_RATE_MAX_RULE_ID = "rate.ascent.max"

# Fixture defaults in meters / second.Both are magnitudes.
DEFAULT_MAX_DESCENT_RATE = 0.5
DEFAULT_MAX_ASCENT_RATE = 0.5

_UNUSABLE_SUMMARY = "heave rate input is not usable"
_DESCENT_EXCEEDED_SUMMARY = "descent rate exceeds max_descent_rate"
_ASCENT_EXCEEDED_SUMMARY = "ascent rate exceeds max_ascent_rate"

_RESULT = safety_rule_result


def _limit_usable(max_rate: float) -> bool:
  return math.isfinite(max_rate) and max_rate >= 0.0


def evaluate_heave_rate_bound_rule(
    depth_rate: float,
    max_descent_rate: float = DEFAULT_MAX_DESCENT_RATE,
    max_ascent_rate: float = DEFAULT_MAX_ASCENT_RATE,
) -> _RESULT.SafetyRuleResult:
  """Evaluate `rate.descent.max` and `rate.ascent.max`.

    Returns one result per call: the first applicable case below.
    1. Non-finite `depth_rate`, `max_descent_rate`, or `max_ascent_rate`, or
       either max negative:
  CRITICAL recommending REJECT with no projection.
       `rule_id` is `rate.ascent.max` only when the ascent max is the sole
       unusable input. Otherwise it is `rate.descent.max`.
    2. `depth_rate > max_descent_rate`: ERROR recommending PROJECT,
       `rate.descent.max`, `projected_value == max_descent_rate`.
    3. `depth_rate < -max_ascent_rate`: ERROR recommending PROJECT,
       `rate.ascent.max`, `projected_value == -max_ascent_rate`.
    4. Otherwise compliant. Equality with either limit is compliant.
  """
  descent_usable = _limit_usable(max_descent_rate)
  ascent_usable = _limit_usable(max_ascent_rate)
  if not math.isfinite(depth_rate) or not descent_usable or not ascent_usable:
    rule_id = (
        ASCENT_RATE_MAX_RULE_ID
        if descent_usable and not ascent_usable
        else DESCENT_RATE_MAX_RULE_ID
    )
    return _RESULT.SafetyRuleResult(
        violated=True,
        rule_id=rule_id,
        severity=_RESULT.SEVERITY_CRITICAL,
        summary=_UNUSABLE_SUMMARY,
        recommended_kind=_RESULT.DECISION_KIND_REJECT,
    )
  if depth_rate > max_descent_rate:
    return _RESULT.SafetyRuleResult(
        violated=True,
        rule_id=DESCENT_RATE_MAX_RULE_ID,
        severity=_RESULT.SEVERITY_ERROR,
        summary=_DESCENT_EXCEEDED_SUMMARY,
        recommended_kind=_RESULT.DECISION_KIND_PROJECT,
        has_projected_value=True,
        projected_value=max_descent_rate,
    )
  if depth_rate < -max_ascent_rate:
    return _RESULT.SafetyRuleResult(
        violated=True,
        rule_id=ASCENT_RATE_MAX_RULE_ID,
        severity=_RESULT.SEVERITY_ERROR,
        summary=_ASCENT_EXCEEDED_SUMMARY,
        recommended_kind=_RESULT.DECISION_KIND_PROJECT,
        has_projected_value=True,
        projected_value=-max_ascent_rate,
    )
  return _RESULT.SafetyRuleResult()
