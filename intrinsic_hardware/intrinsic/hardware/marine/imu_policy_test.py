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

"""Validation tests for the IMU measurement policy."""

import dataclasses
import math
import unittest

from intrinsic.hardware.marine import imu_policy
from intrinsic.hardware.marine import ins_policy
from intrinsic.hardware.marine import measurement_health_policy

from intrinsic.embodiment import frame_policy
from intrinsic.embodiment import stamped_header_policy
from intrinsic.vehicle import vehicle_contract_policy


def _covariance():
  values = [0.0] * vehicle_contract_policy.COVARIANCE_VALUES
  values[imu_policy.ANGULAR_VELOCITY_X_VARIANCE_SLOT] = 0.25
  values[imu_policy.ANGULAR_VELOCITY_Y_VARIANCE_SLOT] = 0.5
  values[imu_policy.ANGULAR_VELOCITY_Z_VARIANCE_SLOT] = 0.125
  values[imu_policy.LINEAR_ACCELERATION_X_VARIANCE_SLOT] = 1.0
  values[imu_policy.LINEAR_ACCELERATION_Y_VARIANCE_SLOT] = 2.0
  values[imu_policy.LINEAR_ACCELERATION_Z_VARIANCE_SLOT] = 4.0
  return tuple(values)


def _valid_imu():
  health = measurement_health_policy.MeasurementHealthView(
      header_present=True,
      frame_id="imu",
      source_time_present=True,
      source_time=(1700000000, 250000000),
      receive_time_present=True,
      receive_time=(1700000001, 0),
      header_validity_present=True,
      header_validity_state=1,
      state_present=True,
      state=1,
      quality_present=True,
      quality=0.75,
      covariance_present=True,
      covariance=_covariance(),
  )
  return imu_policy.ImuMeasurementView(
      health=health,
      angular_velocity_x_present=True,
      angular_velocity_x_rad_s=0.25,
      angular_velocity_y_present=True,
      angular_velocity_y_rad_s=-0.5,
      angular_velocity_z_present=True,
      angular_velocity_z_rad_s=0.125,
      linear_acceleration_x_present=True,
      linear_acceleration_x_m_s2=0.0,
      linear_acceleration_y_present=True,
      linear_acceleration_y_m_s2=0.0,
      linear_acceleration_z_present=True,
      linear_acceleration_z_m_s2=8.0,
      orientation_present=True,
      orientation_w=1.0,
  )


def _health(**kwargs):
  sample = _valid_imu()
  return dataclasses.replace(
      sample, health=dataclasses.replace(sample.health, **kwargs)
  )


class ImuPolicyTest(unittest.TestCase):

  def test_tolerances_match(self):
    self.assertEqual(imu_policy.UNIT_QUATERNION_TOLERANCE, 1e-6)
    self.assertEqual(
        imu_policy.UNIT_QUATERNION_TOLERANCE,
        ins_policy.UNIT_QUATERNION_TOLERANCE,
    )

  def test_empty_sample_is_absent_and_not_an_error(self):
    assessment = imu_policy.assess_imu(imu_policy.ImuMeasurementView())
    self.assertEqual(assessment.error, imu_policy.ImuError.NONE)
    self.assertEqual(
        assessment.state, measurement_health_policy.MeasurementStateKind.ABSENT
    )
    self.assertEqual(
        assessment.header_validity, stamped_header_policy.ValidityKind.ABSENT
    )
    self.assertFalse(assessment.accepted)

  def test_nominal_with_and_without_orientation(self):
    self.assertTrue(imu_policy.assess_imu(_valid_imu()).accepted)
    raw = dataclasses.replace(
        _valid_imu(), orientation_present=False, orientation_w=0.0
    )
    self.assertTrue(imu_policy.assess_imu(raw).accepted)
    zeros = dataclasses.replace(
        _valid_imu(),
        angular_velocity_x_rad_s=0.0,
        angular_velocity_y_rad_s=0.0,
        angular_velocity_z_rad_s=0.0,
        linear_acceleration_x_m_s2=0.0,
        linear_acceleration_y_m_s2=0.0,
        linear_acceleration_z_m_s2=0.0,
    )
    self.assertTrue(imu_policy.assess_imu(zeros).accepted)

  def test_incomplete_triples_and_missing_health(self):
    missing_rate = dataclasses.replace(
        _valid_imu(),
        angular_velocity_z_present=False,
        angular_velocity_z_rad_s=math.nan,
    )
    self.assertEqual(
        imu_policy.assess_imu(missing_rate).error,
        imu_policy.ImuError.ANGULAR_VELOCITY,
    )
    missing_accel = dataclasses.replace(
        _valid_imu(), linear_acceleration_x_present=False
    )
    self.assertEqual(
        imu_policy.assess_imu(missing_accel).error,
        imu_policy.ImuError.LINEAR_ACCELERATION,
    )
    rates_only = imu_policy.ImuMeasurementView(
        angular_velocity_x_present=True,
        angular_velocity_y_present=True,
        angular_velocity_z_present=True,
    )
    self.assertEqual(
        imu_policy.assess_imu(rates_only).error,
        imu_policy.ImuError.MISSING_HEALTH,
    )

  def test_non_finite_payload_does_not_rewrite_state(self):
    rate = dataclasses.replace(_valid_imu(), angular_velocity_y_rad_s=math.inf)
    assessment = imu_policy.assess_imu(rate)
    self.assertEqual(assessment.error, imu_policy.ImuError.NON_FINITE)
    self.assertEqual(
        assessment.state, measurement_health_policy.MeasurementStateKind.VALID
    )
    self.assertEqual(rate.health.state, 1)
    accel = dataclasses.replace(
        _valid_imu(), linear_acceleration_z_m_s2=math.nan
    )
    self.assertEqual(
        imu_policy.assess_imu(accel).error, imu_policy.ImuError.NON_FINITE
    )

  def test_quaternion_tolerance_does_not_renormalize(self):
    halves = dataclasses.replace(
        _valid_imu(),
        orientation_x=0.5,
        orientation_y=0.5,
        orientation_z=0.5,
        orientation_w=0.5,
    )
    self.assertTrue(imu_policy.assess_imu(halves).accepted)
    self.assertEqual(halves.orientation_w, 0.5)
    edge = dataclasses.replace(
        _valid_imu(),
        orientation_w=1.0 + imu_policy.UNIT_QUATERNION_TOLERANCE,
    )
    self.assertTrue(imu_policy.assess_imu(edge).accepted)
    outside = dataclasses.replace(
        _valid_imu(),
        orientation_w=math.nextafter(
            1.0 + imu_policy.UNIT_QUATERNION_TOLERANCE, 2.0
        ),
    )
    self.assertEqual(
        imu_policy.assess_imu(outside).error, imu_policy.ImuError.ORIENTATION
    )
    self.assertEqual(outside.health.state, 1)
    inside_low = 1.0 - imu_policy.UNIT_QUATERNION_TOLERANCE
    while not imu_policy.unit_quaternion(0.0, 0.0, 0.0, inside_low):
      inside_low = math.nextafter(inside_low, 1.0)
    low = dataclasses.replace(_valid_imu(), orientation_w=inside_low)
    self.assertTrue(imu_policy.assess_imu(low).accepted)
    below = dataclasses.replace(
        _valid_imu(), orientation_w=math.nextafter(inside_low, 0.0)
    )
    self.assertEqual(
        imu_policy.assess_imu(below).error, imu_policy.ImuError.ORIENTATION
    )
    flipped = dataclasses.replace(_valid_imu(), orientation_w=-1.0)
    self.assertTrue(imu_policy.assess_imu(flipped).accepted)
    axis = dataclasses.replace(
        _valid_imu(), orientation_x=1.0, orientation_w=0.0
    )
    self.assertTrue(imu_policy.assess_imu(axis).accepted)
    non_unit = dataclasses.replace(_valid_imu(), orientation_w=2.0)
    self.assertEqual(
        imu_policy.assess_imu(non_unit).error, imu_policy.ImuError.ORIENTATION
    )
    nan = dataclasses.replace(_valid_imu(), orientation_x=math.nan)
    self.assertEqual(
        imu_policy.assess_imu(nan).error, imu_policy.ImuError.NON_FINITE
    )

  def test_quality_and_covariance(self):
    self.assertTrue(imu_policy.assess_imu(_health(quality=0.0)).accepted)
    self.assertTrue(imu_policy.assess_imu(_health(quality=1.0)).accepted)
    self.assertEqual(
        imu_policy.assess_imu(_health(quality=math.nextafter(1.0, 2.0))).error,
        imu_policy.ImuError.QUALITY,
    )
    absent = _health(covariance_present=False, covariance=())
    self.assertTrue(imu_policy.assess_imu(absent).accepted)
    zeros = tuple([0.0] * vehicle_contract_policy.COVARIANCE_VALUES)
    self.assertTrue(imu_policy.assess_imu(_health(covariance=zeros)).accepted)
    self.assertTrue(vehicle_contract_policy.is_all_zero_covariance(zeros))
    off = list(_covariance())
    off[1] = 1e-12
    self.assertEqual(
        imu_policy.assess_imu(_health(covariance=tuple(off))).error,
        imu_policy.ImuError.COVARIANCE_SLOTS,
    )
    short = _health(covariance=tuple([0.0] * 35))
    self.assertEqual(
        imu_policy.assess_imu(short).error, imu_policy.ImuError.COVARIANCE
    )

  def test_metadata_and_validity_stay_put(self):
    self.assertEqual(
        imu_policy.assess_imu(_health(frame_id="")).error,
        imu_policy.ImuError.MISSING_FRAME,
    )
    wrong = _health(
        expected_frame_present=True,
        expected_frame=frame_policy.WORLD_ENU_FRAME_ID,
    )
    self.assertEqual(
        imu_policy.assess_imu(wrong).error, imu_policy.ImuError.WRONG_FRAME
    )
    matching = _health(
        frame_id=frame_policy.WORLD_ENU_FRAME_ID,
        expected_frame_present=True,
        expected_frame=frame_policy.WORLD_ENU_FRAME_ID,
    )
    self.assertTrue(imu_policy.assess_imu(matching).accepted)
    reversed_time = _health(receive_time=(1700000000, 249999999))
    self.assertEqual(
        imu_policy.assess_imu(reversed_time).error,
        imu_policy.ImuError.TIME_REVERSAL,
    )
    empty_source = measurement_health_policy.SourceHealthView("", False, 0)
    sourced = _health(sources=(empty_source,))
    self.assertEqual(
        imu_policy.assess_imu(sourced).error, imu_policy.ImuError.SOURCE_ID
    )
    invalid_header = _health(header_validity_state=2)
    assessment = imu_policy.assess_imu(invalid_header)
    self.assertTrue(assessment.accepted)
    self.assertEqual(
        assessment.header_validity, stamped_header_policy.ValidityKind.INVALID
    )
    degraded = _health(state=2, header_validity_present=False)
    degraded_assessment = imu_policy.assess_imu(degraded)
    self.assertEqual(degraded_assessment.error, imu_policy.ImuError.NONE)
    self.assertEqual(
        degraded_assessment.state,
        measurement_health_policy.MeasurementStateKind.DEGRADED,
    )
    self.assertFalse(degraded_assessment.accepted)
    self.assertEqual(degraded.health.state, 2)

  def test_first_defect_wins(self):
    sample = _health(frame_id="")
    sample = dataclasses.replace(sample, angular_velocity_x_present=False)
    self.assertEqual(
        imu_policy.assess_imu(sample).error, imu_policy.ImuError.MISSING_FRAME
    )
    sample = dataclasses.replace(_health(quality=2.0), orientation_w=2.0)
    self.assertEqual(
        imu_policy.assess_imu(sample).error, imu_policy.ImuError.QUALITY
    )
    slotted = list(_covariance())
    slotted[vehicle_contract_policy.covariance_index(0, 1)] = 0.1
    slotted[vehicle_contract_policy.covariance_index(1, 0)] = 0.1
    sample = dataclasses.replace(
        _health(covariance=tuple(slotted)), angular_velocity_z_present=False
    )
    self.assertEqual(
        imu_policy.assess_imu(sample).error,
        imu_policy.ImuError.COVARIANCE_SLOTS,
    )
    sample = dataclasses.replace(
        _valid_imu(),
        angular_velocity_x_present=False,
        linear_acceleration_y_present=False,
        orientation_w=2.0,
    )
    self.assertEqual(
        imu_policy.assess_imu(sample).error,
        imu_policy.ImuError.ANGULAR_VELOCITY,
    )


if __name__ == "__main__":
  unittest.main()
