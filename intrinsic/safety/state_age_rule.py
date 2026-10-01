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

"""Pure state-age gate.

Times are injected by the caller. This function does not read a wall clock,
World, ICON, a HAL, or the snapshot skew assessor.
"""

from intrinsic.embodiment import stamped_header_policy
from intrinsic.safety import safety_rule_result

STATE_AGE_RULE_ID = "state.age"

# (seconds, nanos). nanos must be in [0, 1000000000) to be usable.
StateTime = tuple[int, int]

# Fixture default. An age equal to this bound is compliant.
DEFAULT_MAX_AGE: StateTime = (2, 0)

_UNUSABLE_SUMMARY = "state age input is not usable"
_EXCEEDED_SUMMARY = "state age exceeds max_age"

_RESULT = safety_rule_result
_HEADER = stamped_header_policy


def _age_violation(summary: str) -> _RESULT.SafetyRuleResult:
  return _RESULT.SafetyRuleResult(
      violated=True,
      rule_id=STATE_AGE_RULE_ID,
      severity=_RESULT.SEVERITY_ERROR,
      summary=summary,
      recommended_kind=_RESULT.DECISION_KIND_REJECT,
  )


def _max_age_ok(max_age: StateTime) -> bool:
  seconds, nanos = max_age
  return seconds >= 0 and _HEADER.nanos_in_range(nanos)


def evaluate_state_age_rule(
    query_time: StateTime,
    observation_time: StateTime,
    max_age: StateTime = DEFAULT_MAX_AGE,
) -> _RESULT.SafetyRuleResult:
  """Evaluate `state.age`.

  Equality with `max_age` is compliant. A strictly greater age, a future
  observation stamp, nanos outside [0, 1e9), or a negative max_age is a
  violation at ERROR recommending REJECT.
  """
  query_seconds, query_nanos = query_time
  observation_seconds, observation_nanos = observation_time
  max_seconds, max_nanos = max_age
  if (
      not _HEADER.nanos_in_range(query_nanos)
      or not _HEADER.nanos_in_range(observation_nanos)
      or not _max_age_ok(max_age)
  ):
    return _age_violation(_UNUSABLE_SUMMARY)
  if (observation_seconds, observation_nanos) > (query_seconds, query_nanos):
    return _age_violation(_UNUSABLE_SUMMARY)

  seconds = query_seconds - observation_seconds
  nanos = query_nanos - observation_nanos
  if nanos < 0:
    seconds -= 1
    nanos += _HEADER.NANOS_PER_SECOND
  if (seconds, nanos) > (max_seconds, max_nanos):
    return _age_violation(_EXCEEDED_SUMMARY)
  return _RESULT.SafetyRuleResult()
