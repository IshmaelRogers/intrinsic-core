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

"""Plain-value result of one pure safety rule.

This is not a SafetyDecision and it does not aggregate findings.
"""

from dataclasses import dataclass

# Locked wire numbers from SafetyFindingSeverity and SafetyDecisionKind.
SEVERITY_UNSPECIFIED = 0
SEVERITY_ERROR = 3
SEVERITY_CRITICAL = 4
DECISION_KIND_UNSPECIFIED = 0
DECISION_KIND_REJECT = 3


@dataclass(frozen=True)
class SafetyRuleResult:
  """One rule evaluation.

  `violated` false means the input is compliant and no finding is produced.
  `rule_id` and `summary` are empty when the input is compliant. `severity`
  is a raw SafetyFindingSeverity wire number. `recommended_kind` is a raw
  SafetyDecisionKind wire number: REJECT when violated, UNSPECIFIED otherwise.
  These rules never recommend PROJECT, ACCEPT, ABORT, or SURFACE.
  """

  violated: bool = False
  rule_id: str = ""
  severity: int = 0
  summary: str = ""
  recommended_kind: int = 0
