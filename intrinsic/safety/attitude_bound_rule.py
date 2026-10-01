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

"""Pure pitch and roll attitude gates.

Angles are radians. Each angle is wrapped to (-pi, pi] before it is compared
with its absolute limit. These functions do not read World, ICON, a HAL, or
Gazebo and do not build a DesiredMotion. They only clamp a scalar.
"""

import math

from intrinsic.safety import safety_rule_result

ATTITUDE_PITCH_MAX_RULE_ID = "attitude.pitch.max"
ATTITUDE_ROLL_MAX_RULE_ID = "attitude.roll.max"

# Fixture defaults in radians(30 degrees).
DEFAULT_MAX_PITCH = math.pi / 6.0
DEFAULT_MAX_ROLL = math.pi / 6.0

_UNUSABLE_SUMMARY = "attitude input is not usable"
_PITCH_EXCEEDED_SUMMARY = "pitch magnitude exceeds max_pitch"
_ROLL_EXCEEDED_SUMMARY = "roll magnitude exceeds max_roll"

_RESULT = safety_rule_result


def wrap_to_pi(angle_rad: float) -> float:
  """Wraps `angle_rad` to the equivalent angle in (-pi, pi].

  An angle already in that interval is returned unchanged. Otherwise the
  result is `atan2(sin, cos)`, with -pi mapped to +pi. Non-finite input
  returns NaN.
  """
  if not math.isfinite(angle_rad):
    return math.nan
  if -math.pi < angle_rad <= math.pi:
    return angle_rad
  wrapped = math.atan2(math.sin(angle_rad), math.cos(angle_rad))
  return math.pi if wrapped <= -math.pi else wrapped


def _evaluate_attitude_max_rule(
    angle_rad: float, max_angle: float, rule_id: str, exceeded_summary: str
) -> _RESULT.SafetyRuleResult:
  if (
      not math.isfinite(angle_rad)
      or not math.isfinite(max_angle)
      or max_angle < 0.0
  ):
    return _RESULT.SafetyRuleResult(
        violated=True,
        rule_id=rule_id,
        severity=_RESULT.SEVERITY_CRITICAL,
        summary=_UNUSABLE_SUMMARY,
        recommended_kind=_RESULT.DECISION_KIND_REJECT,
    )
  wrapped = wrap_to_pi(angle_rad)
  if abs(wrapped) <= max_angle:
    return _RESULT.SafetyRuleResult()
  return _RESULT.SafetyRuleResult(
      violated=True,
      rule_id=rule_id,
      severity=_RESULT.SEVERITY_ERROR,
      summary=exceeded_summary,
      recommended_kind=_RESULT.DECISION_KIND_PROJECT,
      has_projected_value=True,
      projected_value=math.copysign(max_angle, wrapped),
  )


def evaluate_attitude_pitch_max_rule(
    pitch_rad: float, max_pitch: float = DEFAULT_MAX_PITCH
) -> _RESULT.SafetyRuleResult:
  """Evaluate `attitude.pitch.max`.

  Non-finite `pitch_rad` or `max_pitch`, or a negative `max_pitch`, is
  CRITICAL recommending REJECT with no projection. `|wrap_to_pi(pitch_rad)|
  <= max_pitch` is compliant. Otherwise ERROR recommending PROJECT with
  `projected_value` equal to `copysign(max_pitch, wrap_to_pi(pitch_rad))`.
  """
  return _evaluate_attitude_max_rule(
      pitch_rad,
      max_pitch,
      ATTITUDE_PITCH_MAX_RULE_ID,
      _PITCH_EXCEEDED_SUMMARY,
  )


def evaluate_attitude_roll_max_rule(
    roll_rad: float, max_roll: float = DEFAULT_MAX_ROLL
) -> _RESULT.SafetyRuleResult:
  """Evaluate `attitude.roll.max`. Same behavior as pitch."""
  return _evaluate_attitude_max_rule(
      roll_rad,
      max_roll,
      ATTITUDE_ROLL_MAX_RULE_ID,
      _ROLL_EXCEEDED_SUMMARY,
  )
