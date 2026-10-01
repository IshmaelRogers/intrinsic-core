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

"""Tests for the energy.reserve rule."""

import math
import unittest

from intrinsic.safety import energy_reserve_rule
from intrinsic.safety import safety_decision_policy
from intrinsic.safety import safety_rule_result

_RULE = energy_reserve_rule
_DECISION = safety_decision_policy
_RESULT = safety_rule_result

_EPS = 1e-6


def _sample(
    available_j,
    required_j,
    energy_known=True,
    prediction_horizon_s=60.0,
    prediction_usable=True,
):
  return _RULE.EnergyReserveSample(
      available_j=available_j,
      required_j=required_j,
      energy_known=energy_known,
      prediction_horizon_s=prediction_horizon_s,
      prediction_usable=prediction_usable,
  )


class EnergyReserveRuleTest(unittest.TestCase):

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
    self.assertEqual(result.rule_id, "energy.reserve")
    self.assertEqual(result.rule_id, _RULE.ENERGY_RESERVE_RULE_ID)
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

  def _expect_below_reserve(self, result, available_j):
    self.assertTrue(result.violated)
    self.assertEqual(result.rule_id, "energy.reserve")
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
    self.assertAlmostEqual(result.projected_value, available_j, places=12)
    self.assertIn("available_j=", result.summary)
    self.assertIn("required_j=", result.summary)

  def test_rule_id_is_locked(self):
    self.assertEqual(_RULE.ENERGY_RESERVE_RULE_ID, "energy.reserve")

  def test_above_required_is_compliant(self):
    self._expect_compliant(
        _RULE.evaluate_energy_reserve_rule(_sample(1000.0 + _EPS, 1000.0))
    )
    self._expect_compliant(
        _RULE.evaluate_energy_reserve_rule(_sample(5000.0, 1000.0))
    )

  def test_equal_to_required_is_compliant(self):
    self._expect_compliant(
        _RULE.evaluate_energy_reserve_rule(_sample(1000.0, 1000.0))
    )
    self._expect_compliant(
        _RULE.evaluate_energy_reserve_rule(_sample(0.0, 0.0))
    )

  def test_below_required_is_rejected_with_both_values(self):
    available = 1000.0 - _EPS
    result = _RULE.evaluate_energy_reserve_rule(_sample(available, 1000.0))
    self._expect_below_reserve(result, available)
    self.assertIn("required_j=1000", result.summary)

  def test_below_required_summary_includes_both_values(self):
    self.assertEqual(
        _RULE.evaluate_energy_reserve_rule(_sample(250.5, 1000.0)).summary,
        "energy below reserve available_j=250.5 required_j=1000",
    )
    self.assertEqual(
        _RULE.evaluate_energy_reserve_rule(_sample(0.0, 10.0)).summary,
        "energy below reserve available_j=0 required_j=10",
    )

  def test_zero_available_against_positive_required_is_below(self):
    self._expect_below_reserve(
        _RULE.evaluate_energy_reserve_rule(_sample(0.0, 10.0)), 0.0
    )

  def test_unknown_energy_is_critical_even_when_numbers_pass(self):
    result = _RULE.evaluate_energy_reserve_rule(
        _sample(5000.0, 1000.0, energy_known=False)
    )
    self._expect_critical(result)
    self.assertIn("unknown energy", result.summary)
    self._expect_critical(
        _RULE.evaluate_energy_reserve_rule(
            _sample(0.0, 1000.0, energy_known=False)
        )
    )

  def test_unusable_prediction_is_critical(self):
    result = _RULE.evaluate_energy_reserve_rule(
        _sample(5000.0, 1000.0, prediction_usable=False)
    )
    self._expect_critical(result)
    self.assertIn("prediction unusable", result.summary)

  def test_non_positive_or_non_finite_horizon_is_critical(self):
    for bad in (0.0, -1.0, -_EPS, math.nan, math.inf, -math.inf):
      self._expect_critical(
          _RULE.evaluate_energy_reserve_rule(
              _sample(5000.0, 1000.0, prediction_horizon_s=bad)
          )
      )

  def test_non_finite_available_or_required_is_critical(self):
    for bad in (math.nan, math.inf, -math.inf):
      self._expect_critical(
          _RULE.evaluate_energy_reserve_rule(_sample(bad, 1000.0))
      )
      self._expect_critical(
          _RULE.evaluate_energy_reserve_rule(_sample(5000.0, bad))
      )

  def test_negative_available_or_required_is_critical(self):
    for available, required in ((-1.0, 1000.0), (-1.0, -2.0), (5000.0, -1.0)):
      self._expect_critical(
          _RULE.evaluate_energy_reserve_rule(_sample(available, required))
      )


if __name__ == "__main__":
  unittest.main()
