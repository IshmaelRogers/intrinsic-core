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

"""Pure minimum-clearance rule over a plain injected sample.

This does not read World, occupancy, or SDF data, call a planner, or rewrite a
path. A sample under the limit is rejected, never clamped or projected.
"""

import dataclasses
import enum
import math

from intrinsic.safety import safety_rule_result

CLEARANCE_MIN_RULE_ID = "clearance.min"
DEFAULT_MIN_CLEARANCE_M = 0.5

_BAD_CONFIG_SUMMARY = "clearance bad config"
_SNAPSHOT_UNUSABLE_SUMMARY = "clearance snapshot unusable"
_UNKNOWN_MAP_SUMMARY = "clearance unknown map"
_UNUSABLE_SUMMARY = "clearance input is not usable"

_RESULT = safety_rule_result


class ClearanceSource(enum.Enum):
  OBSTACLE = 0
  SEAFLOOR = 1
  UNKNOWN_MAP = 2


@dataclasses.dataclass(frozen=True)
class ClearanceSample:
  """Minimum predicted clearance supplied by the caller.

  `frame_id` must be non-empty and is a case-sensitive audit tag only.
  `clearance_m` is meters. `snapshot_usable` false means the snapshot is stale
  or invalid.
  """

  frame_id: str
  source: ClearanceSource
  clearance_m: float
  snapshot_usable: bool = True


def _critical(summary: str) -> _RESULT.SafetyRuleResult:
  return _RESULT.SafetyRuleResult(
      violated=True,
      rule_id=CLEARANCE_MIN_RULE_ID,
      severity=_RESULT.SEVERITY_CRITICAL,
      summary=summary,
      recommended_kind=_RESULT.DECISION_KIND_REJECT,
  )


def evaluate_clearance_rule(
    sample: ClearanceSample,
    min_clearance_m: float = DEFAULT_MIN_CLEARANCE_M,
) -> _RESULT.SafetyRuleResult:
  """Evaluate `clearance.min` for one sample.

  Checked in this order:
    - Empty `frame_id`: CRITICAL, REJECT.
    - `snapshot_usable` false: CRITICAL, REJECT.
    - `UNKNOWN_MAP` source (or any non-`ClearanceSource` value): CRITICAL,
      REJECT. Never treated as clear.
    - Non-finite `clearance_m` or `min_clearance_m`, or `min_clearance_m <= 0`:
      CRITICAL, REJECT.
    - `clearance_m >= min_clearance_m` (equal is compliant): compliant.
    - Otherwise ERROR, REJECT with summary
      `clearance below min source=<obstacle|seafloor> clearance_m=<value>`.
      `has_projected_value` is true and `projected_value` is the observed
      `clearance_m`. This is for audit only. The decision kind is never
      PROJECT.
  """
  if not sample.frame_id:
    return _critical(_BAD_CONFIG_SUMMARY)
  if not sample.snapshot_usable:
    return _critical(_SNAPSHOT_UNUSABLE_SUMMARY)
  if sample.source is ClearanceSource.OBSTACLE:
    source_name = "obstacle"
  elif sample.source is ClearanceSource.SEAFLOOR:
    source_name = "seafloor"
  else:
    return _critical(_UNKNOWN_MAP_SUMMARY)
  if (
      not math.isfinite(sample.clearance_m)
      or not math.isfinite(min_clearance_m)
      or min_clearance_m <= 0.0
  ):
    return _critical(_UNUSABLE_SUMMARY)
  if sample.clearance_m >= min_clearance_m:
    return _RESULT.SafetyRuleResult()
  return _RESULT.SafetyRuleResult(
      violated=True,
      rule_id=CLEARANCE_MIN_RULE_ID,
      severity=_RESULT.SEVERITY_ERROR,
      summary=(
          f"clearance below min source={source_name}"
          f" clearance_m={sample.clearance_m:.9g}"
      ),
      recommended_kind=_RESULT.DECISION_KIND_REJECT,
      has_projected_value=True,
      projected_value=sample.clearance_m,
  )
