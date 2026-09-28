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

"""Validation tests for vehicle state and command contracts."""

import dataclasses
import math
import unittest

from intrinsic.embodiment import capability_policy
from intrinsic.embodiment import frame_policy
from intrinsic.embodiment import stamped_header_policy
from intrinsic.vehicle import vehicle_contract_policy


def _valid_state():
  return vehicle_contract_policy.VehicleStateView(
      validity_present=True,
      validity_state=1,
      frame_id="world_enu",
      pose_present=True,
      position=(1.0, 2.0, -3.0),
      orientation=(0.0, 0.0, 0.0, 1.0),
      twist_present=True,
      twist=(0.5, 0.0, 0.0, 0.0, 0.0, 0.125),
  )


def _valid_motion():
  return vehicle_contract_policy.DesiredMotionView(
      header_present=True,
      validity_present=True,
      validity_state=1,
      frame_id="world_enu",
      objective=vehicle_contract_policy.ObjectiveKind.POSE,
      position=(4.0, 0.0, -2.0),
      orientation=(0.0, 0.0, 0.0, 1.0),
      confidence_present=True,
      confidence=0.5,
      horizon_present=True,
      horizon_seconds=5,
      provenance_present=True,
      model_id="uuv_intent",
  )


def _valid_wrench():
  return vehicle_contract_policy.BodyWrenchView(
      engaged=True,
      validity_present=True,
      validity_state=1,
      frame_id=vehicle_contract_policy.BODY_FRAME_ID,
      force_torque=(1.5, 0.0, 0.0, 0.0, 0.0, 0.25),
  )


class VehicleContractTest(unittest.TestCase):

  def test_empty_state_is_opt_in_and_not_accepted(self):
    assessment = vehicle_contract_policy.assess_vehicle_state(
        vehicle_contract_policy.VehicleStateView()
    )
    self.assertIs(assessment.error, vehicle_contract_policy.ContractError.NONE)
    self.assertIs(
        assessment.validity, stamped_header_policy.ValidityKind.ABSENT
    )
    self.assertFalse(assessment.accepted)

  def test_valid_state_is_accepted(self):
    assessment = vehicle_contract_policy.assess_vehicle_state(_valid_state())
    self.assertIs(assessment.error, vehicle_contract_policy.ContractError.NONE)
    self.assertTrue(assessment.accepted)

  def test_valid_stamp_does_not_repair_nan_or_infinity(self):
    nan_twist = dataclasses.replace(
        _valid_state(), twist=(math.nan, 0.0, 0.0, 0.0, 0.0, 0.0)
    )
    assessment = vehicle_contract_policy.assess_vehicle_state(nan_twist)
    self.assertIs(
        assessment.error, vehicle_contract_policy.ContractError.NON_FINITE
    )
    self.assertIs(assessment.validity, stamped_header_policy.ValidityKind.VALID)
    self.assertFalse(assessment.accepted)
    infinite = dataclasses.replace(
        _valid_state(),
        acceleration_present=True,
        acceleration=(0.0, 0.0, math.inf, 0.0, 0.0, 0.0),
    )
    self.assertIs(
        vehicle_contract_policy.assess_vehicle_state(infinite).error,
        vehicle_contract_policy.ContractError.NON_FINITE,
    )

  def test_invalid_stamp_is_not_rewritten(self):
    state = dataclasses.replace(_valid_state(), validity_state=2)
    assessment = vehicle_contract_policy.assess_vehicle_state(state)
    self.assertIs(assessment.error, vehicle_contract_policy.ContractError.NONE)
    self.assertIs(
        assessment.validity, stamped_header_policy.ValidityKind.INVALID
    )
    self.assertFalse(assessment.accepted)

  def test_absent_validity_is_not_invalid(self):
    state = dataclasses.replace(_valid_state(), validity_present=False)
    assessment = vehicle_contract_policy.assess_vehicle_state(state)
    self.assertIs(
        assessment.validity, stamped_header_policy.ValidityKind.ABSENT
    )
    self.assertIsNot(
        assessment.validity, stamped_header_policy.ValidityKind.INVALID
    )

  def test_quaternion_and_frame_rules(self):
    doubled = dataclasses.replace(
        _valid_state(), orientation=(0.0, 0.0, 0.0, 2.0)
    )
    self.assertIs(
        vehicle_contract_policy.assess_vehicle_state(doubled).error,
        vehicle_contract_policy.ContractError.QUATERNION,
    )
    self.assertFalse(frame_policy.is_normalized(doubled.orientation))
    missing = dataclasses.replace(_valid_state(), frame_id="")
    self.assertIs(
        vehicle_contract_policy.assess_vehicle_state(missing).error,
        vehicle_contract_policy.ContractError.MISSING_FRAME,
    )

  def test_body_twist_is_not_converted_with_the_pose_frame(self):
    body = (1.0, 2.0, 3.0)
    ned = frame_policy.world_vector_enu_to_ned(body)
    self.assertNotEqual(ned[0], body[0])
    state = dataclasses.replace(
        _valid_state(),
        frame_id=frame_policy.WORLD_ENU_FRAME_ID,
        twist=(body[0], body[1], body[2], 0.0, 0.0, 0.0),
    )
    self.assertTrue(
        vehicle_contract_policy.assess_vehicle_state(state).accepted
    )
    self.assertEqual(state.twist[:3], body)
    self.assertFalse(frame_policy.frame_id_matches("VehicleState", "enu"))
    self.assertFalse(
        frame_policy.frame_id_matches(
            vehicle_contract_policy.BODY_FRAME_ID, "enu"
        )
    )

  def test_covariance_unknown_is_absence_not_zeros(self):
    self.assertIs(
        vehicle_contract_policy.assess_covariance(False, ()),
        vehicle_contract_policy.CovarianceError.ABSENT,
    )
    self.assertTrue(
        vehicle_contract_policy.covariance_is_unknown(
            vehicle_contract_policy.CovarianceError.ABSENT
        )
    )
    zeros = (0.0,) * vehicle_contract_policy.COVARIANCE_VALUES
    self.assertIs(
        vehicle_contract_policy.assess_covariance(True, zeros),
        vehicle_contract_policy.CovarianceError.NONE,
    )
    self.assertTrue(vehicle_contract_policy.is_all_zero_covariance(zeros))
    self.assertFalse(
        vehicle_contract_policy.covariance_is_unknown(
            vehicle_contract_policy.assess_covariance(True, zeros)
        )
    )
    self.assertIs(
        vehicle_contract_policy.assess_covariance(True, ()),
        vehicle_contract_policy.CovarianceError.WRONG_LENGTH,
    )

  def test_covariance_shape_and_symmetry(self):
    values = [0.0] * vehicle_contract_policy.COVARIANCE_VALUES
    values[0] = math.nan
    self.assertIs(
        vehicle_contract_policy.assess_covariance(True, values),
        vehicle_contract_policy.CovarianceError.NON_FINITE,
    )
    values[0] = 0.25
    values[1] = 1.0
    self.assertIs(
        vehicle_contract_policy.assess_covariance(True, values),
        vehicle_contract_policy.CovarianceError.ASYMMETRIC,
    )
    values[1] = 1e-12
    self.assertIs(
        vehicle_contract_policy.assess_covariance(True, values),
        vehicle_contract_policy.CovarianceError.NONE,
    )
    values[1] = 1e-6
    self.assertIs(
        vehicle_contract_policy.assess_covariance(True, values),
        vehicle_contract_policy.CovarianceError.ASYMMETRIC,
    )
    self.assertEqual(vehicle_contract_policy.covariance_index(1, 0), 6)
    self.assertEqual(vehicle_contract_policy.covariance_index(5, 5), 35)
    state = dataclasses.replace(
        _valid_state(),
        pose_covariance_present=True,
        pose_covariance=(0.0, 0.0, 0.0),
    )
    self.assertIs(
        vehicle_contract_policy.assess_vehicle_state(state).error,
        vehicle_contract_policy.ContractError.POSE_COVARIANCE,
    )

  def test_source_health_and_navigation_mode(self):
    rejected = dataclasses.replace(
        _valid_state(),
        sources=(
            vehicle_contract_policy.SourceView("dvl", True, 1),
            vehicle_contract_policy.SourceView("", False, 0),
        ),
    )
    self.assertIs(
        vehicle_contract_policy.assess_vehicle_state(rejected).error,
        vehicle_contract_policy.ContractError.SOURCE_ID,
    )
    depth = vehicle_contract_policy.SourceView("depth", False, 0)
    accepted = dataclasses.replace(_valid_state(), sources=(depth,))
    self.assertTrue(
        vehicle_contract_policy.assess_vehicle_state(accepted).accepted
    )
    self.assertIs(
        stamped_header_policy.classify_validity(False, 0),
        stamped_header_policy.ValidityKind.ABSENT,
    )
    self.assertIs(
        vehicle_contract_policy.classify_navigation_mode(100),
        vehicle_contract_policy.NavigationModeKind.UNKNOWN,
    )
    self.assertIsNot(
        vehicle_contract_policy.classify_navigation_mode(100),
        vehicle_contract_policy.NavigationModeKind.FAULTED,
    )
    self.assertIs(
        vehicle_contract_policy.classify_navigation_mode(4),
        vehicle_contract_policy.NavigationModeKind.FAULTED,
    )

  def test_manipulator_capability_fixture_stays_opt_in(self):
    self.assertFalse(
        capability_policy.is_well_known_capability_id(
            "ai.intrinsic.capability.vehicle"
        )
    )
    assessment = capability_policy.assess_declarations(
        capability_policy.MANIPULATOR_CAPABILITY_DECLARATIONS
    )
    self.assertIs(assessment.error, capability_policy.DeclarationError.NONE)
    self.assertFalse(
        capability_policy.declares_capability(
            capability_policy.MANIPULATOR_CAPABILITY_DECLARATIONS,
            "ai.intrinsic.capability.vehicle",
        )
    )

  def test_motion_and_wrench_rules(self):
    self.assertFalse(
        vehicle_contract_policy.assess_desired_motion(
            vehicle_contract_policy.DesiredMotionView()
        ).accepted
    )
    self.assertFalse(
        vehicle_contract_policy.assess_body_wrench(
            vehicle_contract_policy.BodyWrenchView()
        ).accepted
    )
    self.assertTrue(
        vehicle_contract_policy.assess_desired_motion(_valid_motion()).accepted
    )
    twist = dataclasses.replace(
        _valid_motion(),
        objective=vehicle_contract_policy.ObjectiveKind.TWIST,
        frame_id=vehicle_contract_policy.BODY_FRAME_ID,
        provenance_present=False,
    )
    self.assertTrue(
        vehicle_contract_policy.assess_desired_motion(twist).accepted
    )
    world_twist = dataclasses.replace(
        twist, frame_id=frame_policy.WORLD_ENU_FRAME_ID
    )
    self.assertIs(
        vehicle_contract_policy.assess_desired_motion(world_twist).error,
        vehicle_contract_policy.ContractError.BODY_FRAME,
    )
    header_only = vehicle_contract_policy.DesiredMotionView(
        header_present=True, validity_present=True, validity_state=1
    )
    self.assertIs(
        vehicle_contract_policy.assess_desired_motion(header_only).error,
        vehicle_contract_policy.ContractError.OBJECTIVE,
    )
    confident = dataclasses.replace(_valid_motion(), confidence=1.5)
    self.assertIs(
        vehicle_contract_policy.assess_desired_motion(confident).error,
        vehicle_contract_policy.ContractError.CONFIDENCE,
    )
    nan_confidence = dataclasses.replace(_valid_motion(), confidence=math.nan)
    self.assertIs(
        vehicle_contract_policy.assess_desired_motion(nan_confidence).error,
        vehicle_contract_policy.ContractError.CONFIDENCE,
    )
    zero_confidence = dataclasses.replace(_valid_motion(), confidence=0.0)
    self.assertTrue(
        vehicle_contract_policy.assess_desired_motion(zero_confidence).accepted
    )
    negative_horizon = dataclasses.replace(_valid_motion(), horizon_seconds=-1)
    self.assertIs(
        vehicle_contract_policy.assess_desired_motion(negative_horizon).error,
        vehicle_contract_policy.ContractError.HORIZON,
    )
    empty_model = dataclasses.replace(_valid_motion(), model_id="")
    self.assertIs(
        vehicle_contract_policy.assess_desired_motion(empty_model).error,
        vehicle_contract_policy.ContractError.PROVENANCE,
    )
    trajectory = dataclasses.replace(
        _valid_motion(),
        objective=vehicle_contract_policy.ObjectiveKind.TRAJECTORY,
        trajectory_id="",
        provenance_present=False,
    )
    self.assertIs(
        vehicle_contract_policy.assess_desired_motion(trajectory).error,
        vehicle_contract_policy.ContractError.TRAJECTORY_ID,
    )
    named = dataclasses.replace(trajectory, trajectory_id="traj-1", frame_id="")
    self.assertTrue(
        vehicle_contract_policy.assess_desired_motion(named).accepted
    )
    neutral = dataclasses.replace(
        _valid_wrench(), force_torque=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    )
    self.assertTrue(
        vehicle_contract_policy.assess_body_wrench(neutral).accepted
    )
    world_wrench = dataclasses.replace(
        _valid_wrench(), frame_id=frame_policy.WORLD_ENU_FRAME_ID
    )
    self.assertIs(
        vehicle_contract_policy.assess_body_wrench(world_wrench).error,
        vehicle_contract_policy.ContractError.BODY_FRAME,
    )
    nan_wrench = dataclasses.replace(
        _valid_wrench(), force_torque=(math.nan, 0.0, 0.0, 0.0, 0.0, 0.0)
    )
    self.assertIs(
        vehicle_contract_policy.assess_body_wrench(nan_wrench).error,
        vehicle_contract_policy.ContractError.NON_FINITE,
    )


if __name__ == "__main__":
  unittest.main()
