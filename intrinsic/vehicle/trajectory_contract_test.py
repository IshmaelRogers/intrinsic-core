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

"""Validation tests for the vehicle trajectory contract."""

import dataclasses
import math
import unittest

from intrinsic.embodiment import frame_policy
from intrinsic.embodiment import stamped_header_policy
from intrinsic.vehicle import trajectory_contract_policy
from intrinsic.vehicle import vehicle_contract_policy


def _sample(seconds, nanos, x, twist_present=True):
  return trajectory_contract_policy.TrajectorySampleView(
      time_present=True,
      seconds=seconds,
      nanos=nanos,
      position=(x, 2.0, -3.0),
      orientation=(0.0, 0.0, 0.0, 1.0),
      twist_present=twist_present,
      twist=(0.5, 0.0, 0.0, 0.0, 0.0, 0.125),
  )


def _valid():
  return trajectory_contract_policy.VehicleTrajectoryView(
      header_present=True,
      validity_present=True,
      validity_state=1,
      frame_id="world_enu",
      trajectory_id="traj_alpha",
      samples=(
          _sample(1700000010, 0, 1.0),
          _sample(1700000011, 500000000, 1.5),
      ),
      provenance_present=True,
      model_id="uuv_planner",
  )


class TrajectoryContractTest(unittest.TestCase):

  def test_empty_trajectory_is_not_engaged(self):
    assessment = trajectory_contract_policy.assess_vehicle_trajectory(
        trajectory_contract_policy.VehicleTrajectoryView()
    )
    self.assertIs(
        assessment.error,
        trajectory_contract_policy.TrajectoryContractError.NONE,
    )
    self.assertIs(
        assessment.validity, stamped_header_policy.ValidityKind.ABSENT
    )
    self.assertFalse(assessment.accepted)
    self.assertFalse(
        trajectory_contract_policy.trajectory_engaged(
            trajectory_contract_policy.VehicleTrajectoryView()
        )
    )

  def test_two_sample_trajectory_is_accepted(self):
    assessment = trajectory_contract_policy.assess_vehicle_trajectory(_valid())
    self.assertIs(
        assessment.error,
        trajectory_contract_policy.TrajectoryContractError.NONE,
    )
    self.assertIs(assessment.validity, stamped_header_policy.ValidityKind.VALID)
    self.assertTrue(assessment.accepted)

  def test_metadata_presence_does_not_change_acceptance(self):
    with_metadata = dataclasses.replace(_valid(), metadata_present=True)
    without_metadata = dataclasses.replace(
        with_metadata, metadata_present=False
    )
    self.assertTrue(
        trajectory_contract_policy.assess_vehicle_trajectory(
            with_metadata
        ).accepted
    )
    self.assertTrue(
        trajectory_contract_policy.assess_vehicle_trajectory(
            without_metadata
        ).accepted
    )

  def test_single_sample_is_monotonic(self):
    trajectory = dataclasses.replace(
        _valid(), samples=(_sample(10, 0, 1.0),), provenance_present=False
    )
    self.assertTrue(
        trajectory_contract_policy.assess_vehicle_trajectory(
            trajectory
        ).accepted
    )

  def test_equal_and_decreasing_times_are_non_monotonic(self):
    first, second = _valid().samples
    equal = dataclasses.replace(
        _valid(),
        samples=(
            first,
            dataclasses.replace(second, seconds=first.seconds, nanos=0),
        ),
    )
    self.assertIs(
        trajectory_contract_policy.assess_vehicle_trajectory(equal).error,
        trajectory_contract_policy.TrajectoryContractError.NON_MONOTONIC_TIME,
    )
    decreasing = dataclasses.replace(
        _valid(),
        samples=(
            first,
            dataclasses.replace(
                second, seconds=first.seconds - 1, nanos=999999999
            ),
        ),
    )
    self.assertIs(
        trajectory_contract_policy.assess_vehicle_trajectory(decreasing).error,
        trajectory_contract_policy.TrajectoryContractError.NON_MONOTONIC_TIME,
    )
    later = dataclasses.replace(
        _valid(),
        samples=(
            first,
            dataclasses.replace(second, seconds=first.seconds, nanos=1),
        ),
    )
    self.assertTrue(
        trajectory_contract_policy.assess_vehicle_trajectory(later).accepted
    )

  def test_missing_frame_empty_id_and_empty_samples(self):
    missing_frame = dataclasses.replace(_valid(), frame_id="")
    self.assertIs(
        trajectory_contract_policy.assess_vehicle_trajectory(
            missing_frame
        ).error,
        trajectory_contract_policy.TrajectoryContractError.MISSING_FRAME,
    )
    empty_id = dataclasses.replace(_valid(), trajectory_id="")
    self.assertIs(
        trajectory_contract_policy.assess_vehicle_trajectory(empty_id).error,
        trajectory_contract_policy.TrajectoryContractError.TRAJECTORY_ID,
    )
    no_samples = dataclasses.replace(_valid(), samples=())
    self.assertIs(
        trajectory_contract_policy.assess_vehicle_trajectory(no_samples).error,
        trajectory_contract_policy.TrajectoryContractError.EMPTY_SAMPLES,
    )
    header_only = trajectory_contract_policy.VehicleTrajectoryView(
        header_present=True,
        validity_present=True,
        validity_state=1,
        frame_id="world_enu",
    )
    self.assertIs(
        trajectory_contract_policy.assess_vehicle_trajectory(header_only).error,
        trajectory_contract_policy.TrajectoryContractError.TRAJECTORY_ID,
    )

  def test_sample_time_must_be_present_and_in_range(self):
    first, second = _valid().samples
    missing = dataclasses.replace(
        _valid(),
        samples=(dataclasses.replace(first, time_present=False), second),
    )
    self.assertIs(
        trajectory_contract_policy.assess_vehicle_trajectory(missing).error,
        trajectory_contract_policy.TrajectoryContractError.SAMPLE_TIME,
    )
    negative = dataclasses.replace(
        _valid(), samples=(first, dataclasses.replace(second, nanos=-1))
    )
    self.assertIs(
        trajectory_contract_policy.assess_vehicle_trajectory(negative).error,
        trajectory_contract_policy.TrajectoryContractError.SAMPLE_TIME,
    )
    overflow = dataclasses.replace(
        _valid(),
        samples=(
            first,
            dataclasses.replace(
                second, nanos=stamped_header_policy.NANOS_PER_SECOND
            ),
        ),
    )
    self.assertIs(
        trajectory_contract_policy.assess_vehicle_trajectory(overflow).error,
        trajectory_contract_policy.TrajectoryContractError.SAMPLE_TIME,
    )
    at_limit = dataclasses.replace(
        _valid(),
        samples=(
            first,
            dataclasses.replace(
                second, nanos=stamped_header_policy.NANOS_PER_SECOND - 1
            ),
        ),
    )
    self.assertTrue(
        trajectory_contract_policy.assess_vehicle_trajectory(at_limit).accepted
    )

  def test_non_finite_pose_twist_and_acceleration(self):
    first, second = _valid().samples
    nan_pose = dataclasses.replace(
        _valid(),
        samples=(
            dataclasses.replace(first, position=(math.nan, 2.0, -3.0)),
            second,
        ),
    )
    assessment = trajectory_contract_policy.assess_vehicle_trajectory(nan_pose)
    self.assertIs(
        assessment.error,
        trajectory_contract_policy.TrajectoryContractError.NON_FINITE,
    )
    self.assertIs(assessment.validity, stamped_header_policy.ValidityKind.VALID)
    self.assertFalse(assessment.accepted)
    infinite_twist = dataclasses.replace(
        _valid(),
        samples=(
            dataclasses.replace(
                first, twist=(0.5, 0.0, math.inf, 0.0, 0.0, 0.0)
            ),
            second,
        ),
    )
    self.assertIs(
        trajectory_contract_policy.assess_vehicle_trajectory(
            infinite_twist
        ).error,
        trajectory_contract_policy.TrajectoryContractError.NON_FINITE,
    )
    nan_accel = dataclasses.replace(
        _valid(),
        samples=(
            first,
            dataclasses.replace(
                second,
                acceleration_present=True,
                acceleration=(0.0, 0.0, 0.0, math.nan, 0.0, 0.0),
            ),
        ),
    )
    self.assertIs(
        trajectory_contract_policy.assess_vehicle_trajectory(nan_accel).error,
        trajectory_contract_policy.TrajectoryContractError.NON_FINITE,
    )

  def test_bad_quaternion_is_not_renormalized(self):
    first, second = _valid().samples
    scaled = dataclasses.replace(
        _valid(),
        samples=(
            dataclasses.replace(first, orientation=(0.0, 0.0, 0.0, 2.0)),
            second,
        ),
    )
    self.assertIs(
        trajectory_contract_policy.assess_vehicle_trajectory(scaled).error,
        trajectory_contract_policy.TrajectoryContractError.QUATERNION,
    )
    self.assertFalse(frame_policy.is_normalized((0.0, 0.0, 0.0, 2.0)))
    zero = dataclasses.replace(
        _valid(),
        samples=(
            dataclasses.replace(first, orientation=(0.0, 0.0, 0.0, 0.0)),
            second,
        ),
    )
    self.assertIs(
        trajectory_contract_policy.assess_vehicle_trajectory(zero).error,
        trajectory_contract_policy.TrajectoryContractError.QUATERNION,
    )
    nan_w = dataclasses.replace(
        _valid(),
        samples=(
            dataclasses.replace(first, orientation=(math.nan, 0.0, 0.0, 1.0)),
            second,
        ),
    )
    self.assertIs(
        trajectory_contract_policy.assess_vehicle_trajectory(nan_w).error,
        trajectory_contract_policy.TrajectoryContractError.NON_FINITE,
    )

  def test_first_defect_follows_the_locked_order(self):
    empty_id = dataclasses.replace(_valid(), trajectory_id="", samples=())
    self.assertIs(
        trajectory_contract_policy.assess_vehicle_trajectory(empty_id).error,
        trajectory_contract_policy.TrajectoryContractError.TRAJECTORY_ID,
    )
    first, second = _valid().samples
    bad_time = dataclasses.replace(
        _valid(),
        frame_id="",
        samples=(
            dataclasses.replace(first, orientation=(0.0, 0.0, 0.0, 2.0)),
            dataclasses.replace(second, time_present=False),
        ),
    )
    self.assertIs(
        trajectory_contract_policy.assess_vehicle_trajectory(bad_time).error,
        trajectory_contract_policy.TrajectoryContractError.SAMPLE_TIME,
    )
    equal = dataclasses.replace(
        _valid(),
        samples=(
            dataclasses.replace(first, orientation=(0.0, 0.0, 0.0, 2.0)),
            dataclasses.replace(second, seconds=first.seconds, nanos=0),
        ),
    )
    self.assertIs(
        trajectory_contract_policy.assess_vehicle_trajectory(equal).error,
        trajectory_contract_policy.TrajectoryContractError.NON_MONOTONIC_TIME,
    )
    missing_frame = dataclasses.replace(
        _valid(),
        frame_id="",
        samples=(
            dataclasses.replace(
                first,
                orientation=(0.0, 0.0, 0.0, 2.0),
                twist=(math.nan, 0.0, 0.0, 0.0, 0.0, 0.0),
            ),
            second,
        ),
    )
    self.assertIs(
        trajectory_contract_policy.assess_vehicle_trajectory(
            missing_frame
        ).error,
        trajectory_contract_policy.TrajectoryContractError.MISSING_FRAME,
    )
    quaternion_before_twist = dataclasses.replace(
        _valid(),
        samples=(
            dataclasses.replace(
                first,
                orientation=(0.0, 0.0, 0.0, 2.0),
                twist=(math.nan, 0.0, 0.0, 0.0, 0.0, 0.0),
            ),
            second,
        ),
    )
    self.assertIs(
        trajectory_contract_policy.assess_vehicle_trajectory(
            quaternion_before_twist
        ).error,
        trajectory_contract_policy.TrajectoryContractError.QUATERNION,
    )

  def test_tolerances_cost_risk_uncertainty_and_provenance(self):
    zeros = dataclasses.replace(_valid(), tolerances_present=True)
    self.assertTrue(
        trajectory_contract_policy.assess_vehicle_trajectory(zeros).accepted
    )
    negative = dataclasses.replace(zeros, position_tolerance_m=-1.0)
    self.assertIs(
        trajectory_contract_policy.assess_vehicle_trajectory(negative).error,
        trajectory_contract_policy.TrajectoryContractError.TOLERANCE,
    )
    infinite = dataclasses.replace(zeros, orientation_tolerance_rad=math.inf)
    self.assertIs(
        trajectory_contract_policy.assess_vehicle_trajectory(infinite).error,
        trajectory_contract_policy.TrajectoryContractError.TOLERANCE,
    )
    negative_cost = dataclasses.replace(_valid(), cost_present=True, cost=-3.0)
    self.assertTrue(
        trajectory_contract_policy.assess_vehicle_trajectory(
            negative_cost
        ).accepted
    )
    nan_cost = dataclasses.replace(_valid(), cost_present=True, cost=math.nan)
    self.assertIs(
        trajectory_contract_policy.assess_vehicle_trajectory(nan_cost).error,
        trajectory_contract_policy.TrajectoryContractError.COST,
    )
    for risk in (0.0, 1.0):
      accepted = dataclasses.replace(_valid(), risk_present=True, risk=risk)
      self.assertTrue(
          trajectory_contract_policy.assess_vehicle_trajectory(
              accepted
          ).accepted
      )
    for risk in (1.1, -0.1, math.inf):
      rejected = dataclasses.replace(_valid(), risk_present=True, risk=risk)
      self.assertIs(
          trajectory_contract_policy.assess_vehicle_trajectory(rejected).error,
          trajectory_contract_policy.TrajectoryContractError.RISK,
      )
    zero_matrix = dataclasses.replace(
        _valid(),
        uncertainty_present=True,
        uncertainty=(0.0,) * vehicle_contract_policy.COVARIANCE_VALUES,
    )
    self.assertTrue(
        trajectory_contract_policy.assess_vehicle_trajectory(
            zero_matrix
        ).accepted
    )
    short = dataclasses.replace(
        _valid(), uncertainty_present=True, uncertainty=(0.0, 0.0, 0.0)
    )
    self.assertIs(
        trajectory_contract_policy.assess_vehicle_trajectory(short).error,
        trajectory_contract_policy.TrajectoryContractError.UNCERTAINTY,
    )
    nan_values = [0.0] * vehicle_contract_policy.COVARIANCE_VALUES
    nan_values[0] = math.nan
    nan_matrix = dataclasses.replace(
        _valid(), uncertainty_present=True, uncertainty=tuple(nan_values)
    )
    self.assertIs(
        trajectory_contract_policy.assess_vehicle_trajectory(nan_matrix).error,
        trajectory_contract_policy.TrajectoryContractError.UNCERTAINTY,
    )
    asymmetric_values = [0.0] * vehicle_contract_policy.COVARIANCE_VALUES
    asymmetric_values[1] = 1.0
    asymmetric = dataclasses.replace(
        _valid(), uncertainty_present=True, uncertainty=tuple(asymmetric_values)
    )
    self.assertIs(
        trajectory_contract_policy.assess_vehicle_trajectory(asymmetric).error,
        trajectory_contract_policy.TrajectoryContractError.UNCERTAINTY,
    )
    unknown = dataclasses.replace(_valid(), uncertainty_present=False)
    self.assertTrue(
        trajectory_contract_policy.assess_vehicle_trajectory(unknown).accepted
    )
    empty_model = dataclasses.replace(_valid(), model_id="")
    self.assertIs(
        trajectory_contract_policy.assess_vehicle_trajectory(empty_model).error,
        trajectory_contract_policy.TrajectoryContractError.PROVENANCE,
    )
    no_provenance = dataclasses.replace(_valid(), provenance_present=False)
    self.assertTrue(
        trajectory_contract_policy.assess_vehicle_trajectory(
            no_provenance
        ).accepted
    )

  def test_invalid_stamp_is_not_rewritten_and_is_not_accepted(self):
    assessment = trajectory_contract_policy.assess_vehicle_trajectory(
        dataclasses.replace(_valid(), validity_state=2)
    )
    self.assertIs(
        assessment.error,
        trajectory_contract_policy.TrajectoryContractError.NONE,
    )
    self.assertIs(
        assessment.validity, stamped_header_policy.ValidityKind.INVALID
    )
    self.assertFalse(assessment.accepted)

  def test_absent_validity_is_not_invalid(self):
    assessment = trajectory_contract_policy.assess_vehicle_trajectory(
        dataclasses.replace(_valid(), validity_present=False)
    )
    self.assertIs(
        assessment.error,
        trajectory_contract_policy.TrajectoryContractError.NONE,
    )
    self.assertIs(
        assessment.validity, stamped_header_policy.ValidityKind.ABSENT
    )
    self.assertFalse(assessment.accepted)

  def test_body_twist_is_not_converted_with_the_pose_frame(self):
    body = (1.0, 2.0, 3.0)
    ned = frame_policy.world_vector_enu_to_ned(body)
    self.assertNotEqual(ned[0], body[0])
    first, second = _valid().samples
    trajectory = dataclasses.replace(
        _valid(),
        samples=(
            dataclasses.replace(
                first, twist=(body[0], body[1], body[2], 0.0, 0.0, 0.0)
            ),
            second,
        ),
    )
    self.assertTrue(
        trajectory_contract_policy.assess_vehicle_trajectory(
            trajectory
        ).accepted
    )
    self.assertEqual(trajectory.samples[0].twist[:3], body)
    self.assertFalse(frame_policy.frame_id_matches("VehicleTrajectory", "enu"))

  def test_metadata_alone_engages_and_needs_an_id(self):
    trajectory = trajectory_contract_policy.VehicleTrajectoryView(
        metadata_present=True
    )
    self.assertTrue(trajectory_contract_policy.trajectory_engaged(trajectory))
    self.assertIs(
        trajectory_contract_policy.assess_vehicle_trajectory(trajectory).error,
        trajectory_contract_policy.TrajectoryContractError.TRAJECTORY_ID,
    )


if __name__ == "__main__":
  unittest.main()
