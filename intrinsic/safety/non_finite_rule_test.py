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

"""Boundary tests for the state.non_finite rule."""

import math
import unittest

from intrinsic.safety import non_finite_rule
from intrinsic.safety import safety_decision_policy
from intrinsic.vehicle import vehicle_contract_policy

_RULE = non_finite_rule
_DECISION = safety_decision_policy
_VEHICLE = vehicle_contract_policy


class NonFiniteRuleTest(unittest.TestCase):

  def _expect_compliant(self, result):
    self.assertFalse(result.violated)
    self.assertEqual(result.rule_id, "")
    self.assertEqual(result.severity, 0)
    self.assertEqual(result.summary, "")
    self.assertEqual(result.recommended_kind, 0)
    self.assertEqual(
        result.recommended_kind, _DECISION.SafetyDecisionKind.UNSPECIFIED.value
    )

  def _expect_non_finite(self, result):
    self.assertTrue(result.violated)
    self.assertEqual(result.rule_id, "state.non_finite")
    self.assertEqual(result.rule_id, _RULE.STATE_NON_FINITE_RULE_ID)
    self.assertEqual(result.severity, 4)
    self.assertEqual(
        result.severity, _DECISION.SafetyFindingSeverityKind.CRITICAL.value
    )
    self.assertEqual(result.summary, "non-finite state value")
    self.assertEqual(result.recommended_kind, 3)
    self.assertEqual(
        result.recommended_kind, _DECISION.SafetyDecisionKind.REJECT.value
    )
    self.assertNotEqual(
        result.recommended_kind, _DECISION.SafetyDecisionKind.PROJECT.value
    )

  def test_empty_span_is_compliant(self):
    self._expect_compliant(_RULE.evaluate_non_finite_rule(()))

  def test_finite_zeros_are_compliant(self):
    self._expect_compliant(
        _RULE.evaluate_non_finite_rule((0.0, -0.0, 1.0, -2.5))
    )

  def test_nan_positive_inf_and_negative_inf_violate(self):
    self._expect_non_finite(_RULE.evaluate_non_finite_rule((math.nan,)))
    self._expect_non_finite(_RULE.evaluate_non_finite_rule((math.inf,)))
    self._expect_non_finite(_RULE.evaluate_non_finite_rule((-math.inf,)))
    self._expect_non_finite(
        _RULE.evaluate_non_finite_rule((0.0, -0.0, 1.0, math.nan))
    )

  def test_default_body_vector_and_motion_are_compliant(self):
    self._expect_compliant(
        _RULE.evaluate_non_finite_body_vector((0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
    )
    self._expect_compliant(
        _RULE.evaluate_non_finite_desired_motion(_VEHICLE.DesiredMotionView())
    )
    self._expect_compliant(
        _RULE.evaluate_non_finite_vehicle_state(_VEHICLE.VehicleStateView())
    )

  def test_body_vector_non_finite_linear_component_violates(self):
    self._expect_non_finite(
        _RULE.evaluate_non_finite_body_vector(
            (math.nan, 0.0, 0.0, 0.0, 0.0, 0.0)
        )
    )
    self._expect_non_finite(
        _RULE.evaluate_non_finite_body_vector(
            (0.0, 0.0, -math.inf, 0.0, 0.0, 0.0)
        )
    )

  def test_desired_motion_non_finite_linear_component_violates(self):
    motion = _VEHICLE.DesiredMotionView(
        twist=(0.0, math.inf, 0.0, 0.0, 0.0, 0.0)
    )
    self._expect_non_finite(_RULE.evaluate_non_finite_desired_motion(motion))
    nan_position = _VEHICLE.DesiredMotionView(position=(math.nan, 0.0, 0.0))
    self._expect_non_finite(
        _RULE.evaluate_non_finite_desired_motion(nan_position)
    )

  def test_vehicle_state_non_finite_linear_component_violates(self):
    state = _VEHICLE.VehicleStateView(twist=(math.nan, 0.0, 0.0, 0.0, 0.0, 0.0))
    self._expect_non_finite(_RULE.evaluate_non_finite_vehicle_state(state))
    acceleration = _VEHICLE.VehicleStateView(
        acceleration=(0.0, 0.0, -math.inf, 0.0, 0.0, 0.0)
    )
    self._expect_non_finite(
        _RULE.evaluate_non_finite_vehicle_state(acceleration)
    )

  def test_vehicle_state_non_finite_covariance_violates(self):
    state = _VEHICLE.VehicleStateView(
        pose_covariance_present=True, pose_covariance=(math.nan,)
    )
    self._expect_non_finite(_RULE.evaluate_non_finite_vehicle_state(state))

  def test_signed_zero_body_vector_is_compliant(self):
    self._expect_compliant(
        _RULE.evaluate_non_finite_body_vector((-0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
    )


if __name__ == "__main__":
  unittest.main()
