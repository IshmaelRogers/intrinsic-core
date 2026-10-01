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
"""Pure aggregation of already-evaluated SafetyRuleResults.

This is not a SafetyDecision, builds no applied intent or digest, and invents
no rule ids, thresholds, or clamp math.
"""

import dataclasses
import math
from typing import Sequence

from intrinsic.safety import safety_rule_result

CONFLICTING_PROJECTIONS_SUMMARY = "conflicting projections"

_RESULT = safety_rule_result


@dataclasses.dataclass(frozen=True)
class AggregatedSafetyResult:
  """Aggregate of a set of rule results.

  `violated` false means the aggregate is compliant (allow / pass-through);
  the aggregator never sets `recommended_kind` to ACCEPT. `recommended_kind`
  is UNSPECIFIED when compliant, else PROJECT or REJECT only.
  `violated_count` is the number of input results with `violated` true.
  `projection_conflict` is true only when the multi-PROJECT escalation to
  REJECT was applied.
  """

  violated: bool = False
  primary_rule_id: str = ""
  severity: int = 0
  summary: str = ""
  recommended_kind: int = 0
  has_projected_value: bool = False
  projected_value: float = 0.0
  violated_count: int = 0
  projection_conflict: bool = False


def _sort_key(result: _RESULT.SafetyRuleResult):
  """Total order so identical rule ids still sort deterministically."""
  value = float(result.projected_value)
  return (
      result.rule_id.encode("utf-8"),
      result.severity,
      result.recommended_kind,
      result.summary.encode("utf-8"),
      result.has_projected_value,
      math.isnan(value),
      0.0 if math.isnan(value) else value,
  )


def aggregate_safety_rule_results(
    results: Sequence[_RESULT.SafetyRuleResult],
) -> AggregatedSafetyResult:
  """Aggregates `results` without mutating them.

  Compliant inputs are ignored. The violated subset is sorted by `rule_id`
  ascending (byte order), so input order never changes the outcome. Then:
    - `severity` is the maximum severity of the violated subset.
    - The primary is the first finding in sorted order with that severity;
      its `rule_id` and `summary` are used unless a projection conflict
      overrides the summary.
    - Any violated finding with kind REJECT, or a kind other than PROJECT or
      REJECT (fail closed), makes the aggregate REJECT. Only the primary's
      projected value is copied (audit only).
    - Otherwise two or more PROJECT findings make the aggregate REJECT with
      `projection_conflict`, no projected value, and summary
      `conflicting projections`.
    - Otherwise exactly one PROJECT finding makes the aggregate PROJECT with
      that finding's projected value.
  Audit-only projected values on REJECT findings never count as projections.
  """
  violated = sorted((r for r in results if r.violated), key=_sort_key)
  if not violated:
    return AggregatedSafetyResult()

  max_severity = max(r.severity for r in violated)
  primary = next(r for r in violated if r.severity == max_severity)
  projects = [
      r for r in violated if r.recommended_kind == _RESULT.DECISION_KIND_PROJECT
  ]
  any_reject_or_unknown = len(projects) != len(violated)

  summary = primary.summary
  conflict = False
  if any_reject_or_unknown:
    kind = _RESULT.DECISION_KIND_REJECT
    source = primary
  elif len(projects) >= 2:
    kind = _RESULT.DECISION_KIND_REJECT
    source = None
    conflict = True
    summary = CONFLICTING_PROJECTIONS_SUMMARY
  else:
    kind = _RESULT.DECISION_KIND_PROJECT
    source = projects[0]

  has_value = source is not None and source.has_projected_value
  return AggregatedSafetyResult(
      violated=True,
      primary_rule_id=primary.rule_id,
      severity=max_severity,
      summary=summary,
      recommended_kind=kind,
      has_projected_value=has_value,
      projected_value=source.projected_value if has_value else 0.0,
      violated_count=len(violated),
      projection_conflict=conflict,
  )
