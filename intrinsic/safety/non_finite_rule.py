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

"""Pure non-finite gate.

Finite checks use math.isfinite, the Python counterpart of
embodiment::IsFinite. This module does not read World, ICON, a HAL, or a
vehicle assessor.
"""

from collections.abc import Sequence
import math

from intrinsic.safety import safety_rule_result
from intrinsic.vehicle import vehicle_contract_policy

STATE_NON_FINITE_RULE_ID = "state.non_finite"

_NON_FINITE_SUMMARY = "non-finite state value"

_RESULT = safety_rule_result
_VEHICLE = vehicle_contract_policy


def _violation() -> _RESULT.SafetyRuleResult:
  return _RESULT.SafetyRuleResult(
      violated=True,
      rule_id=STATE_NON_FINITE_RULE_ID,
      severity=_RESULT.SEVERITY_CRITICAL,
      summary=_NON_FINITE_SUMMARY,
      recommended_kind=_RESULT.DECISION_KIND_REJECT,
  )


def evaluate_non_finite_rule(
    values: Sequence[float],
) -> _RESULT.SafetyRuleResult:
  """Evaluate `state.non_finite` over a sequence of floats.

  An empty sequence is compliant. Any NaN or ±Inf is CRITICAL recommending
  REJECT. ±0.0 is finite.
  """
  for value in values:
    if not math.isfinite(value):
      return _violation()
  return _RESULT.SafetyRuleResult()


def evaluate_non_finite_body_vector(
    value: _VEHICLE.BodyVector,
) -> _RESULT.SafetyRuleResult:
  """Flatten linear then angular components into the span check."""
  return evaluate_non_finite_rule(value)


def evaluate_non_finite_desired_motion(
    motion: _VEHICLE.DesiredMotionView,
) -> _RESULT.SafetyRuleResult:
  """Flatten position, orientation, twist, and confidence."""
  values = (
      *motion.position,
      *motion.orientation,
      *motion.twist,
      motion.confidence,
  )
  return evaluate_non_finite_rule(values)


def evaluate_non_finite_vehicle_state(
    state: _VEHICLE.VehicleStateView,
) -> _RESULT.SafetyRuleResult:
  """Flatten pose, twist, acceleration, and both covariances."""
  values = (
      *state.position,
      *state.orientation,
      *state.twist,
      *state.acceleration,
      *state.pose_covariance,
      *state.twist_covariance,
  )
  return evaluate_non_finite_rule(values)
