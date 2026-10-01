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

"""Tests for the clearance.min rule."""

import math
import unittest

from intrinsic.safety import clearance_rule
from intrinsic.safety import safety_decision_policy
from intrinsic.safety import safety_rule_result

_RULE = clearance_rule
_DECISION = safety_decision_policy
_RESULT = safety_rule_result
_SOURCE = clearance_rule.ClearanceSource

_EPS = 1e-6
_SOURCES = (
    (_SOURCE.OBSTACLE, "obstacle"),
    (_SOURCE.SEAFLOOR, "seafloor"),
)


def _sample(source, clearance_m, frame_id="map", snapshot_usable=True):
  return _RULE.ClearanceSample(
      frame_id=frame_id,
      source=source,
      clearance_m=clearance_m,
      snapshot_usable=snapshot_usable,
  )


class ClearanceRuleTest(unittest.TestCase):

  def _expect_compliant(self, result):
    self.assertFalse(result.violated)
    self.assertEqual(result.rule_id, "")
    self.assertEqual(result.severity, 0)
    self.assertEqual(result.summary, "")
    self.assertEqual(result.recommended_kind, 0)
    self.assertFalse(result.has_projected_value)
    self.assertEqual(result.projected_value, 0.0)

  def _expect_critical(self, result):
    self.assertTrue(result.violated)
    self.assertEqual(result.rule_id, "clearance.min")
    self.assertEqual(result.rule_id, _RULE.CLEARANCE_MIN_RULE_ID)
    self.assertEqual(result.severity, 4)
    self.assertEqual(
        result.severity, _DECISION.SafetyFindingSeverityKind.CRITICAL.value
    )
    self.assertNotEqual(result.summary, "")
    self.assertEqual(result.recommended_kind, 3)
    self.assertEqual(result.recommended_kind, _RESULT.DECISION_KIND_REJECT)
    self.assertEqual(
        result.recommended_kind, _DECISION.SafetyDecisionKind.REJECT.value
    )
    self.assertFalse(result.has_projected_value)
    self.assertEqual(result.projected_value, 0.0)

  def _expect_below_min(self, result, source_name, clearance_m):
    self.assertTrue(result.violated)
    self.assertEqual(result.rule_id, "clearance.min")
    self.assertEqual(result.severity, 3)
    self.assertEqual(
        result.severity, _DECISION.SafetyFindingSeverityKind.ERROR.value
    )
    self.assertEqual(result.recommended_kind, 3)
    self.assertEqual(
        result.recommended_kind, _DECISION.SafetyDecisionKind.REJECT.value
    )
    self.assertNotEqual(
        result.recommended_kind, _DECISION.SafetyDecisionKind.PROJECT.value
    )
    self.assertTrue(result.has_projected_value)
    self.assertAlmostEqual(result.projected_value, clearance_m, places=12)
    self.assertIn(f"source={source_name}", result.summary)
    self.assertIn("clearance_m=", result.summary)

  def test_rule_id_and_default_are_locked(self):
    self.assertEqual(_RULE.CLEARANCE_MIN_RULE_ID, "clearance.min")
    self.assertEqual(_RULE.DEFAULT_MIN_CLEARANCE_M, 0.5)

  def test_just_above_min_is_compliant(self):
    for source, _ in _SOURCES:
      self._expect_compliant(
          _RULE.evaluate_clearance_rule(_sample(source, 0.5 + _EPS))
      )

  def test_equal_to_min_is_compliant(self):
    for source, _ in _SOURCES:
      self._expect_compliant(
          _RULE.evaluate_clearance_rule(_sample(source, 0.5))
      )

  def test_just_below_min_is_rejected_with_observed_value(self):
    for source, name in _SOURCES:
      clearance = 0.5 - _EPS
      self._expect_below_min(
          _RULE.evaluate_clearance_rule(_sample(source, clearance)),
          name,
          clearance,
      )

  def test_zero_and_negative_clearance_are_below_min(self):
    for source, name in _SOURCES:
      for clearance in (0.0, -1.5):
        self._expect_below_min(
            _RULE.evaluate_clearance_rule(_sample(source, clearance)),
            name,
            clearance,
        )

  def test_custom_min_clearance_is_honored(self):
    for source, name in _SOURCES:
      self._expect_compliant(
          _RULE.evaluate_clearance_rule(_sample(source, 2.0), 2.0)
      )
      self._expect_below_min(
          _RULE.evaluate_clearance_rule(_sample(source, 1.9), 2.0), name, 1.9
      )
      self._expect_compliant(
          _RULE.evaluate_clearance_rule(_sample(source, 0.3), 0.25)
      )

  def test_below_min_summary_includes_source_and_value(self):
    self.assertEqual(
        _RULE.evaluate_clearance_rule(_sample(_SOURCE.OBSTACLE, 0.25)).summary,
        "clearance below min source=obstacle clearance_m=0.25",
    )
    self.assertEqual(
        _RULE.evaluate_clearance_rule(_sample(_SOURCE.SEAFLOOR, 0.1)).summary,
        "clearance below min source=seafloor clearance_m=0.1",
    )

  def test_unknown_map_is_critical_even_when_clearance_would_pass(self):
    result = _RULE.evaluate_clearance_rule(_sample(_SOURCE.UNKNOWN_MAP, 100.0))
    self._expect_critical(result)
    self.assertIn("unknown map", result.summary)
    self._expect_critical(
        _RULE.evaluate_clearance_rule(_sample(_SOURCE.UNKNOWN_MAP, 0.0))
    )

  def test_non_enum_source_fails_closed(self):
    self._expect_critical(_RULE.evaluate_clearance_rule(_sample(99, 100.0)))

  def test_unusable_snapshot_is_critical(self):
    for source in _SOURCE:
      result = _RULE.evaluate_clearance_rule(
          _sample(source, 100.0, snapshot_usable=False)
      )
      self._expect_critical(result)
      self.assertIn("snapshot unusable", result.summary)

  def test_empty_frame_is_critical(self):
    self._expect_critical(
        _RULE.evaluate_clearance_rule(
            _sample(_SOURCE.OBSTACLE, 100.0, frame_id="")
        )
    )

  def test_frame_id_is_an_audit_tag_only(self):
    self._expect_compliant(
        _RULE.evaluate_clearance_rule(
            _sample(_SOURCE.OBSTACLE, 1.0, frame_id="ODOM")
        )
    )

  def test_non_finite_clearance_is_critical(self):
    for bad in (math.nan, math.inf, -math.inf):
      for source, _ in _SOURCES:
        self._expect_critical(
            _RULE.evaluate_clearance_rule(_sample(source, bad))
        )

  def test_invalid_min_clearance_is_critical(self):
    for bad in (0.0, -0.5, -_EPS, math.nan, math.inf, -math.inf):
      for source, _ in _SOURCES:
        self._expect_critical(
            _RULE.evaluate_clearance_rule(_sample(source, 10.0), bad)
        )


if __name__ == "__main__":
  unittest.main()
