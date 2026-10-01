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

"""Boundary tests for the speed.linear.max rule."""

import math
import unittest

from intrinsic.safety import safety_decision_policy
from intrinsic.safety import safety_rule_result
from intrinsic.safety import speed_max_rule

_RULE = speed_max_rule
_DECISION = safety_decision_policy
_RESULT = safety_rule_result


class SpeedMaxRuleTest(unittest.TestCase):

  def _expect_compliant(self, result):
    self.assertFalse(result.violated)
    self.assertEqual(result.rule_id, "")
    self.assertEqual(result.severity, 0)
    self.assertEqual(result.summary, "")
    self.assertEqual(result.recommended_kind, 0)
    self.assertFalse(result.has_projected_value)
    self.assertEqual(result.projected_value, 0.0)

  def _expect_clamp(self, result, limit):
    self.assertTrue(result.violated)
    self.assertEqual(result.rule_id, "speed.linear.max")
    self.assertEqual(result.rule_id, _RULE.SPEED_MAX_RULE_ID)
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

  def _expect_reject_critical(self, result):
    self.assertTrue(result.violated)
    self.assertEqual(result.rule_id, "speed.linear.max")
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

  def test_default_max_linear_speed_is_one_point_five_meters_per_second(self):
    self.assertEqual(_RULE.DEFAULT_MAX_LINEAR_SPEED, 1.5)

  def test_just_inside_is_compliant(self):
    self._expect_compliant(_RULE.evaluate_speed_max_rule(1.499))
    self._expect_compliant(_RULE.evaluate_speed_max_rule(0.0))
    self._expect_compliant(_RULE.evaluate_speed_max_rule(-0.0))

  def test_equal_to_limit_is_compliant(self):
    self._expect_compliant(_RULE.evaluate_speed_max_rule(1.5))
    self._expect_compliant(
        _RULE.evaluate_speed_max_rule(1.5, _RULE.DEFAULT_MAX_LINEAR_SPEED)
    )
    self._expect_compliant(_RULE.evaluate_speed_max_rule(0.0, 0.0))
    self._expect_compliant(_RULE.evaluate_speed_max_rule(0.25, 0.25))

  def test_just_outside_clamps_to_limit(self):
    self._expect_clamp(_RULE.evaluate_speed_max_rule(1.501), 1.5)
    self._expect_clamp(_RULE.evaluate_speed_max_rule(3.0), 1.5)
    self._expect_clamp(_RULE.evaluate_speed_max_rule(0.51, 0.5), 0.5)
    self._expect_clamp(_RULE.evaluate_speed_max_rule(0.001, 0.0), 0.0)

  def test_non_finite_input_rejects(self):
    for value in (math.nan, math.inf, -math.inf):
      self._expect_reject_critical(_RULE.evaluate_speed_max_rule(value))

  def test_negative_input_rejects(self):
    self._expect_reject_critical(_RULE.evaluate_speed_max_rule(-0.001))
    self._expect_reject_critical(_RULE.evaluate_speed_max_rule(-1.0, 10.0))

  def test_non_finite_max_rejects(self):
    for limit in (math.nan, math.inf, -math.inf):
      self._expect_reject_critical(_RULE.evaluate_speed_max_rule(0.1, limit))

  def test_negative_max_rejects(self):
    self._expect_reject_critical(_RULE.evaluate_speed_max_rule(0.1, -0.001))
    self._expect_reject_critical(_RULE.evaluate_speed_max_rule(5.0, -1.0))

  def test_negative_zero_max_is_usable(self):
    self._expect_compliant(_RULE.evaluate_speed_max_rule(0.0, -0.0))


if __name__ == "__main__":
  unittest.main()
