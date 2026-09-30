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

"""Validation tests for the INS solution policy."""

import dataclasses
import math
import unittest

from intrinsic.hardware.marine import ins_policy
from intrinsic.hardware.marine import measurement_health_policy

from intrinsic.embodiment import frame_policy
from intrinsic.embodiment import stamped_header_policy
from intrinsic.vehicle import vehicle_contract_policy


def _covariance():
  values = [0.0] * vehicle_contract_policy.COVARIANCE_VALUES
  values[ins_policy.POSITION_X_VARIANCE_SLOT] = 1.0
  values[ins_policy.POSITION_Y_VARIANCE_SLOT] = 4.0
  values[ins_policy.POSITION_Z_VARIANCE_SLOT] = 0.25
  values[ins_policy.ATTITUDE_X_VARIANCE_SLOT] = 0.0625
  values[ins_policy.ATTITUDE_Y_VARIANCE_SLOT] = 0.125
  values[ins_policy.ATTITUDE_Z_VARIANCE_SLOT] = 0.5
  return tuple(values)


def _valid_ins():
  health = measurement_health_policy.MeasurementHealthView(
      header_present=True,
      frame_id="ins",
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
  return ins_policy.InsSolutionView(
      health=health,
      position_x_present=True,
      position_x_m=12.0,
      position_y_present=True,
      position_y_m=-4.0,
      position_z_present=True,
      position_z_m=0.5,
      orientation_present=True,
      orientation_w=1.0,
      linear_velocity_x_present=True,
      linear_velocity_x_m_s=1.5,
      linear_velocity_y_present=True,
      linear_velocity_y_m_s=0.0,
      linear_velocity_z_present=True,
      linear_velocity_z_m_s=-0.25,
      angular_velocity_x_present=True,
      angular_velocity_x_rad_s=0.0,
      angular_velocity_y_present=True,
      angular_velocity_y_rad_s=0.125,
      angular_velocity_z_present=True,
      angular_velocity_z_rad_s=0.0,
      source_present=True,
      source=1,
  )


def _health(**kwargs):
  sample = _valid_ins()
  return dataclasses.replace(
      sample, health=dataclasses.replace(sample.health, **kwargs)
  )


class InsPolicyTest(unittest.TestCase):

  def test_empty_sample_is_absent_and_not_an_error(self):
    assessment = ins_policy.assess_ins(ins_policy.InsSolutionView())
    self.assertEqual(assessment.error, ins_policy.InsError.NONE)
    self.assertEqual(
        assessment.state, measurement_health_policy.MeasurementStateKind.ABSENT
    )
    self.assertEqual(assessment.source, ins_policy.InsSourceKind.ABSENT)
    self.assertFalse(assessment.accepted)

  def test_nominal_solution_and_optional_twist(self):
    nominal = ins_policy.assess_ins(_valid_ins())
    self.assertTrue(nominal.accepted)
    self.assertEqual(nominal.source, ins_policy.InsSourceKind.VENDOR_INS)
    pose_only = dataclasses.replace(
        _valid_ins(),
        linear_velocity_x_present=False,
        linear_velocity_y_present=False,
        linear_velocity_z_present=False,
        angular_velocity_x_present=False,
        angular_velocity_y_present=False,
        angular_velocity_z_present=False,
        source_present=False,
        source=0,
    )
    pose = ins_policy.assess_ins(pose_only)
    self.assertTrue(pose.accepted)
    self.assertEqual(pose.source, ins_policy.InsSourceKind.ABSENT)
    external = ins_policy.assess_ins(
        dataclasses.replace(_valid_ins(), source=2)
    )
    self.assertTrue(external.accepted)
    self.assertEqual(external.source, ins_policy.InsSourceKind.EXTERNAL_NAV)
    origin = dataclasses.replace(
        _valid_ins(), position_x_m=0.0, position_y_m=0.0, position_z_m=0.0
    )
    self.assertTrue(ins_policy.assess_ins(origin).accepted)
    negative = dataclasses.replace(_valid_ins(), position_z_m=-12.5)
    self.assertTrue(ins_policy.assess_ins(negative).accepted)

  def test_position_and_required_orientation(self):
    missing = dataclasses.replace(
        _valid_ins(), position_y_present=False, position_y_m=math.nan
    )
    self.assertEqual(
        ins_policy.assess_ins(missing).error, ins_policy.InsError.POSITION
    )
    non_finite = dataclasses.replace(_valid_ins(), position_x_m=math.inf)
    assessment = ins_policy.assess_ins(non_finite)
    self.assertEqual(assessment.error, ins_policy.InsError.NON_FINITE)
    self.assertEqual(non_finite.health.state, 1)
    missing_orientation = dataclasses.replace(
        _valid_ins(), orientation_present=False
    )
    self.assertEqual(
        ins_policy.assess_ins(missing_orientation).error,
        ins_policy.InsError.ORIENTATION,
    )
    non_unit = dataclasses.replace(_valid_ins(), orientation_w=2.0)
    self.assertEqual(
        ins_policy.assess_ins(non_unit).error, ins_policy.InsError.ORIENTATION
    )
    self.assertEqual(non_unit.orientation_w, 2.0)
    edge = dataclasses.replace(
        _valid_ins(),
        orientation_w=1.0 + ins_policy.UNIT_QUATERNION_TOLERANCE,
    )
    self.assertTrue(ins_policy.assess_ins(edge).accepted)
    outside = dataclasses.replace(
        _valid_ins(),
        orientation_w=math.nextafter(
            1.0 + ins_policy.UNIT_QUATERNION_TOLERANCE, 2.0
        ),
    )
    self.assertEqual(
        ins_policy.assess_ins(outside).error, ins_policy.InsError.ORIENTATION
    )
    inside_low = 1.0 - ins_policy.UNIT_QUATERNION_TOLERANCE
    while not ins_policy.unit_quaternion(0.0, 0.0, 0.0, inside_low):
      inside_low = math.nextafter(inside_low, 1.0)
    self.assertTrue(
        ins_policy.assess_ins(
            dataclasses.replace(_valid_ins(), orientation_w=inside_low)
        ).accepted
    )
    nan = dataclasses.replace(_valid_ins(), orientation_z=math.nan)
    self.assertEqual(
        ins_policy.assess_ins(nan).error, ins_policy.InsError.NON_FINITE
    )

  def test_partial_velocity_and_source(self):
    linear_partial = dataclasses.replace(
        _valid_ins(), linear_velocity_z_present=False
    )
    self.assertEqual(
        ins_policy.assess_ins(linear_partial).error,
        ins_policy.InsError.LINEAR_VELOCITY,
    )
    angular_partial = dataclasses.replace(
        _valid_ins(),
        angular_velocity_x_present=False,
        angular_velocity_y_present=False,
    )
    self.assertEqual(
        ins_policy.assess_ins(angular_partial).error,
        ins_policy.InsError.ANGULAR_VELOCITY,
    )
    unspecified = dataclasses.replace(_valid_ins(), source=0)
    unspecified_assessment = ins_policy.assess_ins(unspecified)
    self.assertEqual(unspecified_assessment.error, ins_policy.InsError.SOURCE)
    self.assertEqual(
        unspecified_assessment.source, ins_policy.InsSourceKind.UNSPECIFIED
    )
    self.assertEqual(unspecified.source, 0)
    unknown = dataclasses.replace(_valid_ins(), source=99)
    unknown_assessment = ins_policy.assess_ins(unknown)
    self.assertEqual(unknown_assessment.error, ins_policy.InsError.SOURCE)
    self.assertEqual(
        unknown_assessment.source, ins_policy.InsSourceKind.UNRECOGNIZED
    )
    self.assertEqual(unknown.source, 99)

  def test_covariance_metadata_and_first_defect(self):
    absent = _health(covariance_present=False, covariance=())
    self.assertTrue(ins_policy.assess_ins(absent).accepted)
    zeros = tuple([0.0] * vehicle_contract_policy.COVARIANCE_VALUES)
    self.assertTrue(ins_policy.assess_ins(_health(covariance=zeros)).accepted)
    off = list(_covariance())
    off[1] = 1e-12
    off[vehicle_contract_policy.covariance_index(1, 0)] = 1e-12
    self.assertEqual(
        ins_policy.assess_ins(_health(covariance=tuple(off))).error,
        ins_policy.InsError.COVARIANCE_SLOTS,
    )
    self.assertEqual(
        ins_policy.assess_ins(_health(frame_id="")).error,
        ins_policy.InsError.MISSING_FRAME,
    )
    wrong = _health(
        expected_frame_present=True,
        expected_frame=frame_policy.WORLD_NED_FRAME_ID,
    )
    self.assertEqual(
        ins_policy.assess_ins(wrong).error, ins_policy.InsError.WRONG_FRAME
    )
    reversed_time = _health(receive_time=(1699999999, 0))
    self.assertEqual(
        ins_policy.assess_ins(reversed_time).error,
        ins_policy.InsError.TIME_REVERSAL,
    )
    self.assertTrue(ins_policy.assess_ins(_health(quality=0.0)).accepted)
    self.assertTrue(ins_policy.assess_ins(_health(quality=1.0)).accepted)
    invalid_header = _health(header_validity_state=2)
    self.assertTrue(ins_policy.assess_ins(invalid_header).accepted)
    self.assertEqual(
        ins_policy.assess_ins(invalid_header).header_validity,
        stamped_header_policy.ValidityKind.INVALID,
    )
    slotted = list(_covariance())
    slotted[8] = 1e-12
    slotted[vehicle_contract_policy.covariance_index(2, 1)] = 1e-12
    sample = dataclasses.replace(
        _health(covariance=tuple(slotted)), orientation_present=False
    )
    self.assertEqual(
        ins_policy.assess_ins(sample).error,
        ins_policy.InsError.COVARIANCE_SLOTS,
    )
    sample = dataclasses.replace(
        _valid_ins(), position_z_present=False, orientation_present=False
    )
    self.assertEqual(
        ins_policy.assess_ins(sample).error, ins_policy.InsError.POSITION
    )
    sample = dataclasses.replace(
        _valid_ins(), orientation_w=2.0, linear_velocity_y_present=False
    )
    self.assertEqual(
        ins_policy.assess_ins(sample).error, ins_policy.InsError.ORIENTATION
    )
    sample = dataclasses.replace(
        _valid_ins(),
        linear_velocity_z_present=False,
        angular_velocity_x_present=False,
    )
    self.assertEqual(
        ins_policy.assess_ins(sample).error, ins_policy.InsError.LINEAR_VELOCITY
    )
    sample = dataclasses.replace(_valid_ins(), source=99)
    self.assertEqual(
        ins_policy.assess_ins(sample).error, ins_policy.InsError.SOURCE
    )
    self.assertEqual(sample.source, 99)


if __name__ == "__main__":
  unittest.main()
