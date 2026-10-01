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

"""Boundary tests for the altitude.min rule."""

import math
import unittest

from intrinsic.safety import altitude_min_rule
from intrinsic.safety import safety_decision_policy
from intrinsic.safety import safety_rule_result

_RULE = altitude_min_rule
_DECISION = safety_decision_policy
_RESULT = safety_rule_result


class AltitudeMinRuleTest(unittest.TestCase):

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
    self.assertEqual(result.rule_id, "altitude.min")
    self.assertEqual(result.rule_id, _RULE.ALTITUDE_MIN_RULE_ID)
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
    self.assertEqual(result.rule_id, "altitude.min")
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

  def test_default_min_altitude_is_two_meters(self):
    self.assertEqual(_RULE.DEFAULT_MIN_ALTITUDE, 2.0)

  def test_just_above_is_compliant(self):
    self._expect_compliant(_RULE.evaluate_altitude_min_rule(True, 2.001))
    self._expect_compliant(_RULE.evaluate_altitude_min_rule(True, 50.0))

  def test_equal_to_limit_is_compliant(self):
    self._expect_compliant(_RULE.evaluate_altitude_min_rule(True, 2.0))
    self._expect_compliant(
        _RULE.evaluate_altitude_min_rule(
            True, 2.0, _RULE.DEFAULT_MIN_ALTITUDE
        )
    )
    self._expect_compliant(_RULE.evaluate_altitude_min_rule(True, 0.0, 0.0))
    self._expect_compliant(_RULE.evaluate_altitude_min_rule(True, 5.5, 5.5))

  def test_just_below_clamps_to_limit(self):
    self._expect_clamp(_RULE.evaluate_altitude_min_rule(True, 1.999), 2.0)
    self._expect_clamp(_RULE.evaluate_altitude_min_rule(True, 0.0), 2.0)
    self._expect_clamp(_RULE.evaluate_altitude_min_rule(True, -1.0), 2.0)
    self._expect_clamp(_RULE.evaluate_altitude_min_rule(True, 4.9, 5.0), 5.0)

  def test_unknown_altitude_rejects_fail_closed(self):
    self._expect_reject_critical(_RULE.evaluate_altitude_min_rule(False, 100.0))
    self._expect_reject_critical(_RULE.evaluate_altitude_min_rule(False, 0.0))
    self._expect_reject_critical(
        _RULE.evaluate_altitude_min_rule(False, 100.0, 2.0)
    )
    self._expect_reject_critical(
        _RULE.evaluate_altitude_min_rule(False, math.nan, -1.0)
    )

  def test_non_finite_altitude_rejects(self):
    for altitude in (math.nan, math.inf, -math.inf):
      self._expect_reject_critical(
          _RULE.evaluate_altitude_min_rule(True, altitude)
      )

  def test_bad_min_altitude_rejects(self):
    for min_altitude in (math.nan, math.inf, -math.inf, -0.001):
      self._expect_reject_critical(
          _RULE.evaluate_altitude_min_rule(True, 10.0, min_altitude)
      )


if __name__ == "__main__":
  unittest.main()
