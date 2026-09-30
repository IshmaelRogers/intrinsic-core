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

"""Validation tests for the DVL measurement policy."""

import dataclasses
import math
import unittest

from intrinsic.hardware.marine import dvl_policy
from intrinsic.hardware.marine import measurement_health_policy

from intrinsic.embodiment import frame_policy
from intrinsic.embodiment import stamped_header_policy
from intrinsic.vehicle import vehicle_contract_policy


def _linear_covariance():
  values = [0.0] * vehicle_contract_policy.COVARIANCE_VALUES
  values[vehicle_contract_policy.covariance_index(0, 0)] = 0.25
  values[vehicle_contract_policy.covariance_index(1, 1)] = 0.25
  values[vehicle_contract_policy.covariance_index(2, 2)] = 0.25
  return tuple(values)


def _valid_bottom():
  health = measurement_health_policy.MeasurementHealthView(
      header_present=True,
      frame_id="sensor",
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
      covariance=_linear_covariance(),
  )
  return dvl_policy.DvlMeasurementView(
      health=health,
      mode_present=True,
      mode=1,
      velocity_x_present=True,
      velocity_x_m_s=0.5,
      velocity_y_present=True,
      velocity_y_m_s=-0.25,
      velocity_z_present=True,
      velocity_z_m_s=0.0,
      bottom_lock_present=True,
      bottom_lock=True,
      altitude_present=True,
      altitude_m=10.0,
  )


class DvlPolicyTest(unittest.TestCase):

  def test_empty_sample_is_absent_and_not_an_error(self):
    assessment = dvl_policy.assess_dvl(dvl_policy.DvlMeasurementView())
    self.assertEqual(assessment.error, dvl_policy.DvlError.NONE)
    self.assertEqual(
        assessment.state, measurement_health_policy.MeasurementStateKind.ABSENT
    )
    self.assertEqual(assessment.mode, dvl_policy.DvlModeKind.ABSENT)
    self.assertEqual(
        assessment.header_validity, stamped_header_policy.ValidityKind.ABSENT
    )
    self.assertFalse(assessment.accepted)

  def test_nominal_bottom_track_is_accepted(self):
    assessment = dvl_policy.assess_dvl(_valid_bottom())
    self.assertEqual(assessment.error, dvl_policy.DvlError.NONE)
    self.assertEqual(
        assessment.state, measurement_health_policy.MeasurementStateKind.VALID
    )
    self.assertEqual(assessment.mode, dvl_policy.DvlModeKind.BOTTOM_TRACK)
    self.assertTrue(assessment.accepted)

  def test_water_track_with_altitude_is_accepted(self):
    sample = dataclasses.replace(
        _valid_bottom(), mode=2, bottom_lock_present=False, bottom_lock=False
    )
    assessment = dvl_policy.assess_dvl(sample)
    self.assertEqual(assessment.error, dvl_policy.DvlError.NONE)
    self.assertEqual(assessment.mode, dvl_policy.DvlModeKind.WATER_TRACK)
    self.assertTrue(assessment.accepted)
    unlocked = dataclasses.replace(
        sample, bottom_lock_present=True, bottom_lock=False
    )
    self.assertTrue(dvl_policy.assess_dvl(unlocked).accepted)

  def test_mode_absence_is_distinct_from_unspecified(self):
    self.assertEqual(
        dvl_policy.classify_dvl_mode(False, 0), dvl_policy.DvlModeKind.ABSENT
    )
    self.assertEqual(
        dvl_policy.classify_dvl_mode(True, 0),
        dvl_policy.DvlModeKind.UNSPECIFIED,
    )
    self.assertEqual(
        dvl_policy.classify_dvl_mode(True, 100),
        dvl_policy.DvlModeKind.UNRECOGNIZED,
    )
    absent = dataclasses.replace(_valid_bottom(), mode_present=False, mode=0)
    self.assertEqual(
        dvl_policy.assess_dvl(absent).error, dvl_policy.DvlError.MODE
    )
    unspecified = dataclasses.replace(_valid_bottom(), mode=0)
    self.assertEqual(
        dvl_policy.assess_dvl(unspecified).mode,
        dvl_policy.DvlModeKind.UNSPECIFIED,
    )
    unknown = dataclasses.replace(_valid_bottom(), mode=100)
    assessment = dvl_policy.assess_dvl(unknown)
    self.assertEqual(assessment.error, dvl_policy.DvlError.MODE)
    self.assertEqual(assessment.mode, dvl_policy.DvlModeKind.UNRECOGNIZED)
    self.assertEqual(unknown.mode, 100)

  def test_velocity_must_be_present_and_finite(self):
    missing = dataclasses.replace(
        _valid_bottom(),
        velocity_z_present=False,
        velocity_x_m_s=math.nan,
    )
    self.assertEqual(
        dvl_policy.assess_dvl(missing).error, dvl_policy.DvlError.VELOCITY
    )
    nan = dataclasses.replace(_valid_bottom(), velocity_y_m_s=math.nan)
    assessment = dvl_policy.assess_dvl(nan)
    self.assertEqual(assessment.error, dvl_policy.DvlError.NON_FINITE)
    self.assertEqual(
        assessment.state, measurement_health_policy.MeasurementStateKind.VALID
    )
    self.assertEqual(nan.health.state, 1)
    zero = dataclasses.replace(
        _valid_bottom(),
        velocity_x_m_s=0.0,
        velocity_y_m_s=0.0,
        velocity_z_m_s=0.0,
    )
    self.assertTrue(dvl_policy.assess_dvl(zero).accepted)
    large = dataclasses.replace(_valid_bottom(), velocity_x_m_s=1.0e6)
    self.assertTrue(dvl_policy.assess_dvl(large).accepted)

  def test_quality_bounds_and_lock_combinations(self):
    low = dataclasses.replace(
        _valid_bottom(),
        health=dataclasses.replace(_valid_bottom().health, quality=0.0),
    )
    self.assertTrue(dvl_policy.assess_dvl(low).accepted)
    high = dataclasses.replace(
        _valid_bottom(),
        health=dataclasses.replace(_valid_bottom().health, quality=1.0),
    )
    self.assertTrue(dvl_policy.assess_dvl(high).accepted)
    above = dataclasses.replace(
        _valid_bottom(),
        health=dataclasses.replace(
            _valid_bottom().health, quality=math.nextafter(1.0, 2.0)
        ),
    )
    self.assertEqual(
        dvl_policy.assess_dvl(above).error, dvl_policy.DvlError.QUALITY
    )
    lock_loss = dataclasses.replace(
        _valid_bottom(),
        bottom_lock=False,
        health=dataclasses.replace(_valid_bottom().health, state=3),
    )
    loss = dvl_policy.assess_dvl(lock_loss)
    self.assertEqual(loss.error, dvl_policy.DvlError.NONE)
    self.assertEqual(
        loss.state, measurement_health_policy.MeasurementStateKind.INVALID
    )
    self.assertFalse(loss.accepted)
    inconsistent = dataclasses.replace(_valid_bottom(), bottom_lock=False)
    self.assertEqual(
        dvl_policy.assess_dvl(inconsistent).error, dvl_policy.DvlError.LOCK
    )
    absent_lock = dataclasses.replace(
        _valid_bottom(), bottom_lock_present=False
    )
    self.assertTrue(dvl_policy.assess_dvl(absent_lock).accepted)

  def test_altitude_is_optional_non_negative_on_either_track(self):
    absent = dataclasses.replace(_valid_bottom(), altitude_present=False)
    self.assertTrue(dvl_policy.assess_dvl(absent).accepted)
    zero = dataclasses.replace(_valid_bottom(), altitude_m=0.0)
    self.assertTrue(dvl_policy.assess_dvl(zero).accepted)
    negative = dataclasses.replace(
        _valid_bottom(), altitude_m=math.nextafter(0.0, -1.0)
    )
    self.assertEqual(
        dvl_policy.assess_dvl(negative).error, dvl_policy.DvlError.ALTITUDE
    )
    water = dataclasses.replace(
        _valid_bottom(),
        mode=2,
        bottom_lock_present=False,
        altitude_m=4.5,
    )
    self.assertTrue(dvl_policy.assess_dvl(water).accepted)

  def test_covariance_absence_zero_and_angular_slots(self):
    absent_health = dataclasses.replace(
        _valid_bottom().health, covariance_present=False, covariance=()
    )
    absent = dataclasses.replace(_valid_bottom(), health=absent_health)
    self.assertTrue(dvl_policy.assess_dvl(absent).accepted)
    zeros = tuple([0.0] * vehicle_contract_policy.COVARIANCE_VALUES)
    zero_health = dataclasses.replace(_valid_bottom().health, covariance=zeros)
    self.assertTrue(
        dvl_policy.assess_dvl(
            dataclasses.replace(_valid_bottom(), health=zero_health)
        ).accepted
    )
    short = tuple([0.0] * 35)
    short_health = dataclasses.replace(_valid_bottom().health, covariance=short)
    self.assertEqual(
        dvl_policy.assess_dvl(
            dataclasses.replace(_valid_bottom(), health=short_health)
        ).error,
        dvl_policy.DvlError.COVARIANCE,
    )
    angular = list(_linear_covariance())
    angular[dvl_policy.ANGULAR_VARIANCE_SLOTS[0]] = 1e-15
    angled = dataclasses.replace(
        _valid_bottom(),
        health=dataclasses.replace(
            _valid_bottom().health, covariance=tuple(angular)
        ),
    )
    self.assertEqual(
        dvl_policy.assess_dvl(angled).error,
        dvl_policy.DvlError.ANGULAR_COVARIANCE,
    )

  def test_metadata_defects_use_health_order(self):
    missing = dataclasses.replace(
        _valid_bottom(),
        health=dataclasses.replace(_valid_bottom().health, frame_id=""),
    )
    self.assertEqual(
        dvl_policy.assess_dvl(missing).error, dvl_policy.DvlError.MISSING_FRAME
    )
    wrong_health = dataclasses.replace(
        _valid_bottom().health,
        expected_frame_present=True,
        expected_frame=frame_policy.WORLD_ENU_FRAME_ID,
    )
    self.assertEqual(
        dvl_policy.assess_dvl(
            dataclasses.replace(_valid_bottom(), health=wrong_health)
        ).error,
        dvl_policy.DvlError.WRONG_FRAME,
    )
    reversed_health = dataclasses.replace(
        _valid_bottom().health, receive_time=(1700000000, 249999999)
    )
    self.assertEqual(
        dvl_policy.assess_dvl(
            dataclasses.replace(_valid_bottom(), health=reversed_health)
        ).error,
        dvl_policy.DvlError.TIME_REVERSAL,
    )

  def test_missing_health_precedes_payload(self):
    payload = dvl_policy.DvlMeasurementView(
        velocity_x_present=True,
        velocity_x_m_s=1.0,
        mode_present=True,
        mode=1,
    )
    self.assertEqual(
        dvl_policy.assess_dvl(payload).error, dvl_policy.DvlError.MISSING_HEALTH
    )
    self.assertEqual(
        stamped_header_policy.classify_validity(True, 3),
        stamped_header_policy.ValidityKind.UNSPECIFIED,
    )

  def test_first_defect_wins(self):
    sample = dataclasses.replace(
        _valid_bottom(),
        mode_present=False,
        health=dataclasses.replace(_valid_bottom().health, frame_id=""),
    )
    self.assertEqual(
        dvl_policy.assess_dvl(sample).error, dvl_policy.DvlError.MISSING_FRAME
    )
    sample = dataclasses.replace(
        _valid_bottom(),
        mode=0,
        health=dataclasses.replace(_valid_bottom().health, quality=2.0),
    )
    self.assertEqual(
        dvl_policy.assess_dvl(sample).error, dvl_policy.DvlError.QUALITY
    )
    sample = dataclasses.replace(
        _valid_bottom(), mode=100, velocity_x_present=False
    )
    self.assertEqual(
        dvl_policy.assess_dvl(sample).error, dvl_policy.DvlError.MODE
    )
    sample = dataclasses.replace(
        _valid_bottom(), velocity_z_m_s=math.nan, bottom_lock=False
    )
    self.assertEqual(
        dvl_policy.assess_dvl(sample).error, dvl_policy.DvlError.NON_FINITE
    )
    sample = dataclasses.replace(
        _valid_bottom(), bottom_lock=False, altitude_m=-1.0
    )
    self.assertEqual(
        dvl_policy.assess_dvl(sample).error, dvl_policy.DvlError.LOCK
    )


if __name__ == "__main__":
  unittest.main()
