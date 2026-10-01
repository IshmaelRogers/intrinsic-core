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

"""Pure energy-reserve rule over a plain injected sample.

This does not predict energy, model a battery or state of charge, integrate
power over a trajectory, call a planner, or rewrite a path. A sample under the
required energy is rejected, never projected.
"""

import dataclasses
import math

from intrinsic.safety import safety_rule_result

ENERGY_RESERVE_RULE_ID = "energy.reserve"

_UNKNOWN_ENERGY_SUMMARY = "unknown energy"
_PREDICTION_UNUSABLE_SUMMARY = "prediction unusable"
_UNUSABLE_SUMMARY = "energy input is not usable"

_RESULT = safety_rule_result


@dataclasses.dataclass(frozen=True)
class EnergyReserveSample:
  """Energy reserve inputs supplied by the caller.

  Energy is in Joules and `prediction_horizon_s` is in seconds. The caller folds
  any configured reserve and fallback energy into `required_j`.
  `energy_known` false means the available energy is unknown.
  `prediction_usable` false means the prediction is stale, expired, or invalid.
  """

  available_j: float
  required_j: float
  prediction_horizon_s: float
  energy_known: bool = True
  prediction_usable: bool = True


def _critical(summary: str) -> _RESULT.SafetyRuleResult:
  return _RESULT.SafetyRuleResult(
      violated=True,
      rule_id=ENERGY_RESERVE_RULE_ID,
      severity=_RESULT.SEVERITY_CRITICAL,
      summary=summary,
      recommended_kind=_RESULT.DECISION_KIND_REJECT,
  )


def evaluate_energy_reserve_rule(
    sample: EnergyReserveSample,
) -> _RESULT.SafetyRuleResult:
  """Evaluate `energy.reserve` for one sample.

  Checked in this order:
    - `energy_known` false: CRITICAL, REJECT. Fails closed even when the
      numbers would pass.
    - `prediction_usable` false: CRITICAL, REJECT.
    - Non-finite `available_j`, `required_j`, or `prediction_horizon_s`, or
      `available_j < 0`, `required_j < 0`, or `prediction_horizon_s <= 0`:
      CRITICAL, REJECT.
    - `available_j >= required_j` (equal is compliant): compliant.
    - Otherwise ERROR, REJECT with summary
      `energy below reserve available_j=<a> required_j=<r>`.
      `has_projected_value` is true and `projected_value` is `available_j`.
      This is for audit only. The decision kind is never PROJECT.
  """
  if not sample.energy_known:
    return _critical(_UNKNOWN_ENERGY_SUMMARY)
  if not sample.prediction_usable:
    return _critical(_PREDICTION_UNUSABLE_SUMMARY)
  if (
      not math.isfinite(sample.available_j)
      or not math.isfinite(sample.required_j)
      or not math.isfinite(sample.prediction_horizon_s)
      or sample.available_j < 0.0
      or sample.required_j < 0.0
      or sample.prediction_horizon_s <= 0.0
  ):
    return _critical(_UNUSABLE_SUMMARY)
  if sample.available_j >= sample.required_j:
    return _RESULT.SafetyRuleResult()
  return _RESULT.SafetyRuleResult(
      violated=True,
      rule_id=ENERGY_RESERVE_RULE_ID,
      severity=_RESULT.SEVERITY_ERROR,
      summary=(
          f"energy below reserve available_j={sample.available_j:.9g}"
          f" required_j={sample.required_j:.9g}"
      ),
      recommended_kind=_RESULT.DECISION_KIND_REJECT,
      has_projected_value=True,
      projected_value=sample.available_j,
  )
