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

"""Boundary tests for the attitude.pitch.max and attitude.roll.max rules."""

import math
import unittest

from intrinsic.safety import attitude_bound_rule
from intrinsic.safety import safety_decision_policy
from intrinsic.safety import safety_rule_result

_RULE = attitude_bound_rule
_DECISION = safety_decision_policy
_RESULT = safety_rule_result


class AttitudeBoundRuleTest(unittest.TestCase):

  def _expect_compliant(self, result):
    self.assertFalse(result.violated)
    self.assertEqual(result.rule_id, "")
    self.assertEqual(result.severity, 0)
    self.assertEqual(result.summary, "")
    self.assertEqual(result.recommended_kind, 0)
    self.assertFalse(result.has_projected_value)
    self.assertEqual(result.projected_value, 0.0)

  def _expect_clamp(self, result, rule_id, limit):
    self.assertTrue(result.violated)
    self.assertEqual(result.rule_id, rule_id)
    self.assertEqual(result.severity, 3)
    self.assertEqual(
        result.severity, _DECISION.SafetyFindingSeverityKind.ERROR.value
    )
    self.assertNotEqual(result.summary, "")
    self.assertEqual(result.recommended_kind, 2)
    self.assertEqual(result.recommended_kind, _RESULT.DECISION_KIND_PROJECT)
    self.assertEqual(
        result.recommended_kind, _DECISION.SafetyDecisionKind.PROJECT.value
    )
    self.assertTrue(result.has_projected_value)
    self.assertEqual(result.projected_value, limit)

  def _expect_reject_critical(self, result, rule_id):
    self.assertTrue(result.violated)
    self.assertEqual(result.rule_id, rule_id)
    self.assertEqual(result.severity, 4)
    self.assertEqual(
        result.severity, _DECISION.SafetyFindingSeverityKind.CRITICAL.value
    )
    self.assertNotEqual(result.summary, "")
    self.assertEqual(result.recommended_kind, 3)
    self.assertEqual(
        result.recommended_kind, _DECISION.SafetyDecisionKind.REJECT.value
    )
    self.assertFalse(result.has_projected_value)
    self.assertEqual(result.projected_value, 0.0)

  def test_rule_ids_and_defaults_are_locked(self):
    self.assertEqual(_RULE.ATTITUDE_PITCH_MAX_RULE_ID, "attitude.pitch.max")
    self.assertEqual(_RULE.ATTITUDE_ROLL_MAX_RULE_ID, "attitude.roll.max")
    self.assertEqual(_RULE.DEFAULT_MAX_PITCH, math.pi / 6.0)
    self.assertEqual(_RULE.DEFAULT_MAX_ROLL, math.pi / 6.0)

  def test_wrap_to_pi_maps_into_half_open_interval(self):
    self.assertEqual(_RULE.wrap_to_pi(0.0), 0.0)
    self.assertEqual(_RULE.wrap_to_pi(1.0), 1.0)
    self.assertEqual(_RULE.wrap_to_pi(-1.0), -1.0)
    self.assertEqual(_RULE.wrap_to_pi(math.pi), math.pi)
    self.assertEqual(_RULE.wrap_to_pi(-math.pi), math.pi)
    self.assertAlmostEqual(
        _RULE.wrap_to_pi(math.pi + 0.1), -math.pi + 0.1, places=12
    )
    self.assertAlmostEqual(
        _RULE.wrap_to_pi(-math.pi - 0.1), math.pi - 0.1, places=12
    )
    self.assertAlmostEqual(_RULE.wrap_to_pi(2.0 * math.pi), 0.0, places=12)
    self.assertAlmostEqual(
        _RULE.wrap_to_pi(2.0 * math.pi + 0.1), 0.1, places=12
    )
    # -3*pi is on the branch cut, so rounding may land on either end.
    self.assertAlmostEqual(
        abs(_RULE.wrap_to_pi(-3.0 * math.pi)), math.pi, places=12
    )
    self.assertAlmostEqual(
        _RULE.wrap_to_pi(7.0 * math.pi + 0.5), -math.pi + 0.5, places=9
    )

  def test_wrap_to_pi_non_finite_is_nan(self):
    for value in (math.nan, math.inf, -math.inf):
      self.assertTrue(math.isnan(_RULE.wrap_to_pi(value)))

  def test_pitch_just_inside_is_compliant(self):
    limit = _RULE.DEFAULT_MAX_PITCH
    self._expect_compliant(_RULE.evaluate_attitude_pitch_max_rule(limit - 1e-9))
    self._expect_compliant(
        _RULE.evaluate_attitude_pitch_max_rule(-(limit - 1e-9))
    )
    self._expect_compliant(_RULE.evaluate_attitude_pitch_max_rule(0.0))
    self._expect_compliant(_RULE.evaluate_attitude_pitch_max_rule(-0.0))

  def test_pitch_equal_to_limit_is_compliant(self):
    limit = _RULE.DEFAULT_MAX_PITCH
    self._expect_compliant(_RULE.evaluate_attitude_pitch_max_rule(limit))
    self._expect_compliant(_RULE.evaluate_attitude_pitch_max_rule(-limit))
    self._expect_compliant(_RULE.evaluate_attitude_pitch_max_rule(0.25, 0.25))
    self._expect_compliant(_RULE.evaluate_attitude_pitch_max_rule(-0.25, 0.25))
    self._expect_compliant(_RULE.evaluate_attitude_pitch_max_rule(0.0, 0.0))

  def test_pitch_just_outside_clamps_with_sign(self):
    limit = _RULE.DEFAULT_MAX_PITCH
    rule_id = "attitude.pitch.max"
    self._expect_clamp(
        _RULE.evaluate_attitude_pitch_max_rule(limit + 1e-6), rule_id, limit
    )
    self._expect_clamp(
        _RULE.evaluate_attitude_pitch_max_rule(-(limit + 1e-6)), rule_id, -limit
    )
    self._expect_clamp(
        _RULE.evaluate_attitude_pitch_max_rule(1.0), rule_id, limit
    )
    self._expect_clamp(
        _RULE.evaluate_attitude_pitch_max_rule(-1.0), rule_id, -limit
    )
    self._expect_clamp(
        _RULE.evaluate_attitude_pitch_max_rule(0.3, 0.25), rule_id, 0.25
    )
    self._expect_clamp(
        _RULE.evaluate_attitude_pitch_max_rule(-0.3, 0.25), rule_id, -0.25
    )
    self._expect_clamp(
        _RULE.evaluate_attitude_pitch_max_rule(0.001, 0.0), rule_id, 0.0
    )

  def test_pitch_wraps_before_compare(self):
    limit = _RULE.DEFAULT_MAX_PITCH
    rule_id = "attitude.pitch.max"
    self._expect_compliant(
        _RULE.evaluate_attitude_pitch_max_rule(2.0 * math.pi + 0.1)
    )
    self._expect_compliant(
        _RULE.evaluate_attitude_pitch_max_rule(-2.0 * math.pi - 0.1)
    )
    self._expect_clamp(
        _RULE.evaluate_attitude_pitch_max_rule(math.pi + 0.1), rule_id, -limit
    )
    self._expect_clamp(
        _RULE.evaluate_attitude_pitch_max_rule(-math.pi - 0.1), rule_id, limit
    )
    self._expect_clamp(
        _RULE.evaluate_attitude_pitch_max_rule(math.pi), rule_id, limit
    )
    self._expect_clamp(
        _RULE.evaluate_attitude_pitch_max_rule(-math.pi), rule_id, limit
    )
    self._expect_compliant(
        _RULE.evaluate_attitude_pitch_max_rule(math.pi, math.pi)
    )
    self._expect_compliant(
        _RULE.evaluate_attitude_pitch_max_rule(-math.pi, math.pi)
    )
    self._expect_compliant(
        _RULE.evaluate_attitude_pitch_max_rule(math.pi + 0.1, math.pi)
    )

  def test_pitch_non_finite_input_rejects(self):
    for value in (math.nan, math.inf, -math.inf):
      self._expect_reject_critical(
          _RULE.evaluate_attitude_pitch_max_rule(value), "attitude.pitch.max"
      )

  def test_pitch_non_finite_max_rejects(self):
    for limit in (math.nan, math.inf, -math.inf):
      self._expect_reject_critical(
          _RULE.evaluate_attitude_pitch_max_rule(0.1, limit),
          "attitude.pitch.max",
      )

  def test_pitch_negative_max_rejects(self):
    self._expect_reject_critical(
        _RULE.evaluate_attitude_pitch_max_rule(0.1, -0.001),
        "attitude.pitch.max",
    )
    self._expect_reject_critical(
        _RULE.evaluate_attitude_pitch_max_rule(0.0, -1.0), "attitude.pitch.max"
    )

  def test_pitch_negative_zero_max_is_usable(self):
    self._expect_compliant(_RULE.evaluate_attitude_pitch_max_rule(0.0, -0.0))

  def test_roll_just_inside_is_compliant(self):
    limit = _RULE.DEFAULT_MAX_ROLL
    self._expect_compliant(_RULE.evaluate_attitude_roll_max_rule(limit - 1e-9))
    self._expect_compliant(
        _RULE.evaluate_attitude_roll_max_rule(-(limit - 1e-9))
    )
    self._expect_compliant(_RULE.evaluate_attitude_roll_max_rule(0.0))
    self._expect_compliant(_RULE.evaluate_attitude_roll_max_rule(-0.0))

  def test_roll_equal_to_limit_is_compliant(self):
    limit = _RULE.DEFAULT_MAX_ROLL
    self._expect_compliant(_RULE.evaluate_attitude_roll_max_rule(limit))
    self._expect_compliant(_RULE.evaluate_attitude_roll_max_rule(-limit))
    self._expect_compliant(_RULE.evaluate_attitude_roll_max_rule(0.25, 0.25))
    self._expect_compliant(_RULE.evaluate_attitude_roll_max_rule(-0.25, 0.25))
    self._expect_compliant(_RULE.evaluate_attitude_roll_max_rule(0.0, 0.0))

  def test_roll_just_outside_clamps_with_sign(self):
    limit = _RULE.DEFAULT_MAX_ROLL
    rule_id = "attitude.roll.max"
    self._expect_clamp(
        _RULE.evaluate_attitude_roll_max_rule(limit + 1e-6), rule_id, limit
    )
    self._expect_clamp(
        _RULE.evaluate_attitude_roll_max_rule(-(limit + 1e-6)), rule_id, -limit
    )
    self._expect_clamp(
        _RULE.evaluate_attitude_roll_max_rule(1.0), rule_id, limit
    )
    self._expect_clamp(
        _RULE.evaluate_attitude_roll_max_rule(-1.0), rule_id, -limit
    )
    self._expect_clamp(
        _RULE.evaluate_attitude_roll_max_rule(0.3, 0.25), rule_id, 0.25
    )
    self._expect_clamp(
        _RULE.evaluate_attitude_roll_max_rule(-0.3, 0.25), rule_id, -0.25
    )
    self._expect_clamp(
        _RULE.evaluate_attitude_roll_max_rule(0.001, 0.0), rule_id, 0.0
    )

  def test_roll_wraps_before_compare(self):
    limit = _RULE.DEFAULT_MAX_ROLL
    rule_id = "attitude.roll.max"
    self._expect_compliant(
        _RULE.evaluate_attitude_roll_max_rule(2.0 * math.pi + 0.1)
    )
    self._expect_compliant(
        _RULE.evaluate_attitude_roll_max_rule(-2.0 * math.pi - 0.1)
    )
    self._expect_clamp(
        _RULE.evaluate_attitude_roll_max_rule(math.pi + 0.1), rule_id, -limit
    )
    self._expect_clamp(
        _RULE.evaluate_attitude_roll_max_rule(-math.pi - 0.1), rule_id, limit
    )
    self._expect_clamp(
        _RULE.evaluate_attitude_roll_max_rule(math.pi), rule_id, limit
    )
    self._expect_clamp(
        _RULE.evaluate_attitude_roll_max_rule(-math.pi), rule_id, limit
    )
    self._expect_compliant(
        _RULE.evaluate_attitude_roll_max_rule(math.pi, math.pi)
    )
    self._expect_compliant(
        _RULE.evaluate_attitude_roll_max_rule(-math.pi, math.pi)
    )
    self._expect_compliant(
        _RULE.evaluate_attitude_roll_max_rule(math.pi + 0.1, math.pi)
    )

  def test_roll_non_finite_input_rejects(self):
    for value in (math.nan, math.inf, -math.inf):
      self._expect_reject_critical(
          _RULE.evaluate_attitude_roll_max_rule(value), "attitude.roll.max"
      )

  def test_roll_non_finite_max_rejects(self):
    for limit in (math.nan, math.inf, -math.inf):
      self._expect_reject_critical(
          _RULE.evaluate_attitude_roll_max_rule(0.1, limit), "attitude.roll.max"
      )

  def test_roll_negative_max_rejects(self):
    self._expect_reject_critical(
        _RULE.evaluate_attitude_roll_max_rule(0.1, -0.001), "attitude.roll.max"
    )
    self._expect_reject_critical(
        _RULE.evaluate_attitude_roll_max_rule(0.0, -1.0), "attitude.roll.max"
    )

  def test_roll_negative_zero_max_is_usable(self):
    self._expect_compliant(_RULE.evaluate_attitude_roll_max_rule(0.0, -0.0))

  def test_pitch_and_roll_use_their_own_rule_ids(self):
    self._expect_clamp(
        _RULE.evaluate_attitude_pitch_max_rule(1.0),
        "attitude.pitch.max",
        _RULE.DEFAULT_MAX_PITCH,
    )
    self._expect_clamp(
        _RULE.evaluate_attitude_roll_max_rule(1.0),
        "attitude.roll.max",
        _RULE.DEFAULT_MAX_ROLL,
    )


if __name__ == "__main__":
  unittest.main()
