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

"""Validation tests for the surface fix policy."""

import dataclasses
import math
import unittest

from intrinsic.hardware.marine import measurement_health_policy
from intrinsic.hardware.marine import surface_fix_policy

from intrinsic.embodiment import frame_policy
from intrinsic.vehicle import vehicle_contract_policy

_ERROR = surface_fix_policy.SurfaceFixError
_SOURCE = surface_fix_policy.SurfaceFixSourceKind


def _covariance():
  values = [0.0] * vehicle_contract_policy.COVARIANCE_VALUES
  values[surface_fix_policy.POSITION_X_VARIANCE_SLOT] = 1.0
  values[surface_fix_policy.POSITION_Y_VARIANCE_SLOT] = 4.0
  values[surface_fix_policy.POSITION_Z_VARIANCE_SLOT] = 0.25
  return tuple(values)


def _valid_fix():
  health = measurement_health_policy.MeasurementHealthView(
      header_present=True,
      frame_id="gnss",
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
  return surface_fix_policy.SurfaceFixView(
      health=health,
      position_x_present=True,
      position_x_m=12.5,
      position_y_present=True,
      position_y_m=-3.25,
      position_z_present=True,
      position_z_m=1.0,
      source_present=True,
      source=1,
      satellite_count_present=True,
      satellite_count=12,
  )


def _health(**kwargs):
  return dataclasses.replace(_valid_fix().health, **kwargs)


def _assess(**kwargs):
  return surface_fix_policy.assess_surface_fix(
      dataclasses.replace(_valid_fix(), **kwargs)
  )


class SurfaceFixPolicyTest(unittest.TestCase):

  def test_empty_sample_is_absent_and_not_an_error(self):
    assessment = surface_fix_policy.assess_surface_fix(
        surface_fix_policy.SurfaceFixView()
    )
    self.assertEqual(assessment.error, _ERROR.NONE)
    self.assertEqual(
        assessment.state, measurement_health_policy.MeasurementStateKind.ABSENT
    )
    self.assertEqual(assessment.source, _SOURCE.ABSENT)
    self.assertFalse(assessment.accepted)

  def test_nominal_gnss_and_acoustic_are_accepted(self):
    gnss = surface_fix_policy.assess_surface_fix(_valid_fix())
    self.assertEqual(gnss.error, _ERROR.NONE)
    self.assertEqual(gnss.source, _SOURCE.GNSS)
    self.assertTrue(gnss.accepted)
    acoustic = _assess(
        source=2,
        satellite_count_present=False,
        beacon_count_present=True,
        beacon_count=4,
    )
    self.assertEqual(acoustic.source, _SOURCE.ACOUSTIC)
    self.assertTrue(acoustic.accepted)
    other = _assess(source=3)
    self.assertEqual(other.source, _SOURCE.OTHER)
    self.assertTrue(other.accepted)

  def test_zero_and_negative_position_are_supplied_values(self):
    zero = _assess(position_x_m=0.0, position_y_m=0.0, position_z_m=0.0)
    self.assertTrue(zero.accepted)
    negative = _assess(position_z_m=-40.0)
    self.assertTrue(negative.accepted)

  def test_incomplete_position_is_rejected(self):
    for field in (
        "position_x_present",
        "position_y_present",
        "position_z_present",
    ):
      sample = _assess(**{field: False})
      self.assertEqual(sample.error, _ERROR.POSITION)
      self.assertFalse(sample.accepted)
    self.assertEqual(
        _assess(
            position_x_present=False,
            position_y_present=False,
            position_z_present=False,
        ).error,
        _ERROR.POSITION,
    )

  def test_non_finite_position_is_rejected(self):
    for field in ("position_x_m", "position_y_m", "position_z_m"):
      for value in (math.nan, math.inf, -math.inf):
        self.assertEqual(_assess(**{field: value}).error, _ERROR.NON_FINITE)

  def test_source_absent_unspecified_and_unknown_are_rejected(self):
    absent = _assess(source_present=False, source=0)
    self.assertEqual(absent.error, _ERROR.SOURCE)
    self.assertEqual(absent.source, _SOURCE.ABSENT)
    unspecified = _assess(source=0)
    self.assertEqual(unspecified.error, _ERROR.SOURCE)
    self.assertEqual(unspecified.source, _SOURCE.UNSPECIFIED)
    unknown = _assess(source=99)
    self.assertEqual(unknown.error, _ERROR.SOURCE)
    self.assertEqual(unknown.source, _SOURCE.UNRECOGNIZED)
    self.assertFalse(unknown.accepted)

  def test_valid_state_with_incomplete_fix_is_not_rewritten(self):
    assessment = _assess(position_z_present=False)
    self.assertEqual(assessment.error, _ERROR.POSITION)
    self.assertEqual(
        assessment.state, measurement_health_policy.MeasurementStateKind.VALID
    )
    self.assertFalse(assessment.accepted)
    no_source = _assess(source_present=False)
    self.assertEqual(
        no_source.state, measurement_health_policy.MeasurementStateKind.VALID
    )
    self.assertFalse(no_source.accepted)

  def test_invalid_fix_is_present_not_accepted_and_not_rewritten(self):
    invalid = _assess(
        health=_health(state=3), source=0, satellite_count_present=False
    )
    self.assertEqual(invalid.error, _ERROR.SOURCE)
    self.assertEqual(
        invalid.state, measurement_health_policy.MeasurementStateKind.INVALID
    )
    self.assertFalse(invalid.accepted)
    lock_loss = _assess(health=_health(state=3))
    self.assertEqual(lock_loss.error, _ERROR.NONE)
    self.assertEqual(
        lock_loss.state, measurement_health_policy.MeasurementStateKind.INVALID
    )
    self.assertFalse(lock_loss.accepted)

  def test_degraded_is_not_accepted(self):
    degraded = _assess(health=_health(state=2))
    self.assertEqual(degraded.error, _ERROR.NONE)
    self.assertFalse(degraded.accepted)

  def test_counts_are_optional_and_zero_is_legal(self):
    none = _assess(satellite_count_present=False)
    self.assertTrue(none.accepted)
    zero = _assess(satellite_count=0, beacon_count_present=True, beacon_count=0)
    self.assertTrue(zero.accepted)
    self.assertEqual(_assess(satellite_count=-1).error, _ERROR.COUNT)
    self.assertEqual(
        _assess(beacon_count_present=True, beacon_count=-1).error, _ERROR.COUNT
    )

  def test_accuracy_is_optional_finite_and_non_negative(self):
    zero = _assess(
        horizontal_accuracy_present=True,
        horizontal_accuracy_m=0.0,
        vertical_accuracy_present=True,
        vertical_accuracy_m=0.0,
    )
    self.assertTrue(zero.accepted)
    self.assertEqual(
        _assess(
            horizontal_accuracy_present=True, horizontal_accuracy_m=-0.5
        ).error,
        _ERROR.ACCURACY,
    )
    self.assertEqual(
        _assess(vertical_accuracy_present=True, vertical_accuracy_m=-0.5).error,
        _ERROR.ACCURACY,
    )
    for value in (math.nan, math.inf):
      self.assertEqual(
          _assess(
              horizontal_accuracy_present=True, horizontal_accuracy_m=value
          ).error,
          _ERROR.NON_FINITE,
      )
      self.assertEqual(
          _assess(
              vertical_accuracy_present=True, vertical_accuracy_m=value
          ).error,
          _ERROR.NON_FINITE,
      )

  def test_velocity_is_all_three_or_none(self):
    full = _assess(
        velocity_x_present=True,
        velocity_y_present=True,
        velocity_z_present=True,
        velocity_x_m_s=0.5,
    )
    self.assertTrue(full.accepted)
    for partial in (
        {"velocity_x_present": True},
        {"velocity_y_present": True},
        {"velocity_z_present": True},
        {"velocity_x_present": True, "velocity_z_present": True},
    ):
      self.assertEqual(_assess(**partial).error, _ERROR.VELOCITY)
    non_finite = _assess(
        velocity_x_present=True,
        velocity_y_present=True,
        velocity_z_present=True,
        velocity_y_m_s=math.inf,
    )
    self.assertEqual(non_finite.error, _ERROR.NON_FINITE)

  def test_quality_endpoints(self):
    self.assertTrue(_assess(health=_health(quality=0.0)).accepted)
    self.assertTrue(_assess(health=_health(quality=1.0)).accepted)
    self.assertEqual(
        _assess(health=_health(quality=math.nextafter(1.0, 2.0))).error,
        _ERROR.QUALITY,
    )
    self.assertEqual(
        _assess(health=_health(quality=-0.25)).error, _ERROR.QUALITY
    )

  def test_covariance_absence_zero_and_slots(self):
    absent = _assess(health=_health(covariance_present=False, covariance=()))
    self.assertTrue(absent.accepted)
    zeros = tuple([0.0] * vehicle_contract_policy.COVARIANCE_VALUES)
    self.assertTrue(_assess(health=_health(covariance=zeros)).accepted)
    short = _assess(health=_health(covariance=tuple([0.0] * 35)))
    self.assertEqual(short.error, _ERROR.COVARIANCE)
    for row in range(6):
      for column in range(6):
        if row == column and row < 3:
          continue
        values = [0.0] * vehicle_contract_policy.COVARIANCE_VALUES
        values[vehicle_contract_policy.covariance_index(row, column)] = 0.0625
        if row != column:
          values[vehicle_contract_policy.covariance_index(column, row)] = 0.0625
        result = _assess(health=_health(covariance=tuple(values)))
        self.assertIn(
            result.error, (_ERROR.COVARIANCE, _ERROR.COVARIANCE_SLOTS)
        )
        self.assertFalse(result.accepted)
    attitude = list(_covariance())
    attitude[vehicle_contract_policy.covariance_index(3, 3)] = 0.5
    self.assertEqual(
        _assess(health=_health(covariance=tuple(attitude))).error,
        _ERROR.COVARIANCE_SLOTS,
    )

  def test_metadata_defects_use_health_order(self):
    self.assertEqual(
        _assess(health=_health(frame_id="")).error, _ERROR.MISSING_FRAME
    )
    wrong = _assess(
        health=_health(
            expected_frame_present=True,
            expected_frame=frame_policy.WORLD_ENU_FRAME_ID,
        )
    )
    self.assertEqual(wrong.error, _ERROR.WRONG_FRAME)
    reversed_time = _assess(
        health=_health(receive_time=(1700000000, 249999999))
    )
    self.assertEqual(reversed_time.error, _ERROR.TIME_REVERSAL)
    delayed = _assess(health=_health(receive_time=(1700000100, 0)))
    self.assertTrue(delayed.accepted)
    empty_source = measurement_health_policy.SourceHealthView("", False, 0)
    self.assertEqual(
        _assess(health=_health(sources=(empty_source,))).error,
        _ERROR.SOURCE_ID,
    )

  def test_missing_health_is_rejected(self):
    payload_only = surface_fix_policy.SurfaceFixView(
        position_x_present=True, position_x_m=1.0
    )
    self.assertEqual(
        surface_fix_policy.assess_surface_fix(payload_only).error,
        _ERROR.MISSING_HEALTH,
    )

  def test_first_defect_wins(self):
    sample = _assess(
        health=_health(frame_id=""),
        position_x_present=False,
        source=99,
    )
    self.assertEqual(sample.error, _ERROR.MISSING_FRAME)
    position_first = _assess(position_x_present=False, source=99)
    self.assertEqual(position_first.error, _ERROR.POSITION)
    source_first = _assess(source=99, satellite_count=-1)
    self.assertEqual(source_first.error, _ERROR.SOURCE)


if __name__ == "__main__":
  unittest.main()
