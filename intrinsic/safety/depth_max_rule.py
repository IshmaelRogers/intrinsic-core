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

"""Pure maximum-depth gate.

Depth is meters positive down from the surface (0 at the surface). This
function does not read World, ICON, a HAL, or Gazebo and does not plan a
trajectory. It only clamps a scalar.
"""

import math

from intrinsic.safety import safety_rule_result

DEPTH_MAX_RULE_ID = "depth.max"

# Fixture default in meters.
DEFAULT_MAX_DEPTH = 100.0

_UNUSABLE_SUMMARY = "depth input is not usable"
_EXCEEDED_SUMMARY = "depth exceeds max_depth"

_RESULT = safety_rule_result


def evaluate_depth_max_rule(
    depth: float, max_depth: float = DEFAULT_MAX_DEPTH
) -> _RESULT.SafetyRuleResult:
  """Evaluate `depth.max`.

  Non-finite `depth` or `max_depth`, or a negative `max_depth`, is CRITICAL
  recommending REJECT with no projection. `depth <= max_depth` is compliant.
  A greater depth is ERROR recommending PROJECT with `projected_value` equal
  to `max_depth`.
  """
  if (
      not math.isfinite(depth)
      or not math.isfinite(max_depth)
      or max_depth < 0.0
  ):
    return _RESULT.SafetyRuleResult(
        violated=True,
        rule_id=DEPTH_MAX_RULE_ID,
        severity=_RESULT.SEVERITY_CRITICAL,
        summary=_UNUSABLE_SUMMARY,
        recommended_kind=_RESULT.DECISION_KIND_REJECT,
    )
  if depth <= max_depth:
    return _RESULT.SafetyRuleResult()
  return _RESULT.SafetyRuleResult(
      violated=True,
      rule_id=DEPTH_MAX_RULE_ID,
      severity=_RESULT.SEVERITY_ERROR,
      summary=_EXCEEDED_SUMMARY,
      recommended_kind=_RESULT.DECISION_KIND_PROJECT,
      has_projected_value=True,
      projected_value=max_depth,
  )
