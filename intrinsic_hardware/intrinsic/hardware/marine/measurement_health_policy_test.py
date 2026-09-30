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

"""Validation tests for marine measurement health."""

import dataclasses
import math
import unittest

from intrinsic.hardware.marine import measurement_health_policy

from intrinsic.embodiment import frame_policy
from intrinsic.embodiment import stamped_header_policy
from intrinsic.vehicle import vehicle_contract_policy


def _valid_sample():
  return measurement_health_policy.MeasurementHealthView(
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
  )


class MeasurementHealthPolicyTest(unittest.TestCase):

  def test_empty_sample_is_opt_in_and_not_accepted(self):
    assessment = measurement_health_policy.assess_measurement_health(
        measurement_health_policy.MeasurementHealthView()
    )
    self.assertIs(
        assessment.error, measurement_health_policy.MeasurementError.NONE
    )
    self.assertIs(
        assessment.state, measurement_health_policy.MeasurementStateKind.ABSENT
    )
    self.assertIs(
        assessment.header_validity, stamped_header_policy.ValidityKind.ABSENT
    )
    self.assertFalse(assessment.accepted)

  def test_nominal_valid_sample_is_accepted(self):
    assessment = measurement_health_policy.assess_measurement_health(
        _valid_sample()
    )
    self.assertIs(
        assessment.error, measurement_health_policy.MeasurementError.NONE
    )
    self.assertIs(
        assessment.state, measurement_health_policy.MeasurementStateKind.VALID
    )
    self.assertTrue(assessment.accepted)

  def test_local_states_stay_distinct_from_embodiment_validity(self):
    kind = measurement_health_policy.MeasurementStateKind
    classify = measurement_health_policy.classify_measurement_state
    self.assertIs(classify(False, 0), kind.ABSENT)
    self.assertIs(classify(True, 0), kind.UNKNOWN)
    self.assertIs(classify(True, 1), kind.VALID)
    self.assertIs(classify(True, 2), kind.DEGRADED)
    self.assertIs(classify(True, 3), kind.INVALID)
    self.assertIs(classify(True, 100), kind.UNRECOGNIZED)
    self.assertIsNot(classify(True, 0), kind.ABSENT)
    unrecognized = measurement_health_policy.assess_measurement_health(
        dataclasses.replace(_valid_sample(), state=100)
    )
    self.assertIs(unrecognized.state, kind.UNRECOGNIZED)
    self.assertIs(
        unrecognized.error, measurement_health_policy.MeasurementError.NONE
    )
    self.assertFalse(unrecognized.accepted)
    self.assertEqual(dataclasses.replace(_valid_sample(), state=100).state, 100)
    self.assertIs(
        stamped_header_policy.classify_validity(True, 3),
        stamped_header_policy.ValidityKind.UNSPECIFIED,
    )
    self.assertIsNot(
        stamped_header_policy.classify_validity(True, 3),
        stamped_header_policy.ValidityKind.INVALID,
    )

  def test_unknown_degraded_and_invalid_follow_documented_status(self):
    kind = measurement_health_policy.MeasurementStateKind
    unknown = measurement_health_policy.assess_measurement_health(
        dataclasses.replace(_valid_sample(), state=0)
    )
    self.assertIs(
        unknown.error, measurement_health_policy.MeasurementError.NONE
    )
    self.assertIs(unknown.state, kind.UNKNOWN)
    self.assertFalse(unknown.accepted)

    degraded = measurement_health_policy.assess_measurement_health(
        dataclasses.replace(_valid_sample(), state=2)
    )
    self.assertIs(degraded.state, kind.DEGRADED)
    self.assertFalse(degraded.accepted)

    dropout = measurement_health_policy.assess_measurement_health(
        dataclasses.replace(_valid_sample(), state=3)
    )
    self.assertIs(dropout.state, kind.INVALID)
    self.assertIs(
        dropout.error, measurement_health_policy.MeasurementError.NONE
    )
    self.assertFalse(dropout.accepted)

    omitted = measurement_health_policy.assess_measurement_health(
        measurement_health_policy.MeasurementHealthView()
    )
    self.assertIs(omitted.state, kind.ABSENT)
    self.assertFalse(omitted.accepted)

  def test_absent_judgment_is_not_unknown(self):
    assessment = measurement_health_policy.assess_measurement_health(
        dataclasses.replace(_valid_sample(), state_present=False, state=0)
    )
    self.assertIs(
        assessment.state, measurement_health_policy.MeasurementStateKind.ABSENT
    )
    self.assertFalse(assessment.accepted)

  def test_header_validity_does_not_replace_measurement_state(self):
    invalid_header = measurement_health_policy.assess_measurement_health(
        dataclasses.replace(_valid_sample(), header_validity_state=2)
    )
    self.assertTrue(invalid_header.accepted)
    self.assertIs(
        invalid_header.header_validity,
        stamped_header_policy.ValidityKind.INVALID,
    )

    invalid_measurement = measurement_health_policy.assess_measurement_health(
        dataclasses.replace(
            _valid_sample(), header_validity_present=False, state=3
        )
    )
    self.assertFalse(invalid_measurement.accepted)
    self.assertIs(
        invalid_measurement.state,
        measurement_health_policy.MeasurementStateKind.INVALID,
    )
    self.assertIs(
        invalid_measurement.header_validity,
        stamped_header_policy.ValidityKind.ABSENT,
    )

  def test_quality_bounds_are_closed_and_finite(self):
    self.assertTrue(
        measurement_health_policy.assess_measurement_health(
            dataclasses.replace(_valid_sample(), quality=0.0)
        ).accepted
    )
    self.assertTrue(
        measurement_health_policy.assess_measurement_health(
            dataclasses.replace(_valid_sample(), quality=1.0)
        ).accepted
    )
    self.assertTrue(
        measurement_health_policy.assess_measurement_health(
            dataclasses.replace(_valid_sample(), quality_present=False)
        ).accepted
    )
    below = measurement_health_policy.assess_measurement_health(
        dataclasses.replace(_valid_sample(), quality=math.nextafter(0.0, -1.0))
    )
    self.assertIs(
        below.error, measurement_health_policy.MeasurementError.QUALITY
    )
    above = measurement_health_policy.assess_measurement_health(
        dataclasses.replace(_valid_sample(), quality=math.nextafter(1.0, 2.0))
    )
    self.assertIs(
        above.error, measurement_health_policy.MeasurementError.QUALITY
    )
    nan = measurement_health_policy.assess_measurement_health(
        dataclasses.replace(_valid_sample(), quality=math.nan)
    )
    self.assertIs(
        nan.error, measurement_health_policy.MeasurementError.NON_FINITE
    )
    self.assertIs(
        nan.state, measurement_health_policy.MeasurementStateKind.VALID
    )

  def test_frame_must_be_explicit_and_can_be_checked(self):
    missing = measurement_health_policy.assess_measurement_health(
        dataclasses.replace(_valid_sample(), frame_id="")
    )
    self.assertIs(
        missing.error, measurement_health_policy.MeasurementError.MISSING_FRAME
    )
    wrong = measurement_health_policy.assess_measurement_health(
        dataclasses.replace(
            _valid_sample(),
            expected_frame_present=True,
            expected_frame=frame_policy.WORLD_ENU_FRAME_ID,
        )
    )
    self.assertIs(
        wrong.error, measurement_health_policy.MeasurementError.WRONG_FRAME
    )
    matched = measurement_health_policy.assess_measurement_health(
        dataclasses.replace(
            _valid_sample(),
            frame_id=frame_policy.WORLD_ENU_FRAME_ID,
            expected_frame_present=True,
            expected_frame=frame_policy.WORLD_ENU_FRAME_ID,
        )
    )
    self.assertTrue(matched.accepted)
    self.assertTrue(frame_policy.frame_id_matches("world_enu", "enu"))
    self.assertFalse(frame_policy.frame_id_matches("MeasurementHealth", "enu"))
    body = measurement_health_policy.assess_measurement_health(
        dataclasses.replace(_valid_sample(), frame_id="body")
    )
    self.assertTrue(body.accepted)

  def test_source_time_reversal_is_rejected_and_delay_is_not(self):
    equal_time = _valid_sample()
    ordered = measurement_health_policy.assess_measurement_health(
        dataclasses.replace(equal_time, receive_time=equal_time.source_time)
    )
    self.assertTrue(ordered.accepted)
    reversed_sample = measurement_health_policy.assess_measurement_health(
        dataclasses.replace(
            _valid_sample(), receive_time=(1700000000, 249999999)
        )
    )
    self.assertIs(
        reversed_sample.error,
        measurement_health_policy.MeasurementError.TIME_REVERSAL,
    )
    delayed = dataclasses.replace(_valid_sample(), receive_time=(1700000100, 0))
    self.assertTrue(
        measurement_health_policy.assess_measurement_health(delayed).accepted
    )
    age = stamped_header_policy.monotonic_age_seconds(
        delayed.source_time,
        delayed.receive_time,
        stamped_header_policy.CLOCK_DOMAIN_MONOTONIC,
    )
    self.assertIsNotNone(age)
    self.assertGreater(age, 0.75)

  def test_covariance_absence_is_not_zero(self):
    absent = _valid_sample()
    self.assertTrue(
        measurement_health_policy.assess_measurement_health(absent).accepted
    )
    self.assertTrue(
        vehicle_contract_policy.covariance_is_unknown(
            vehicle_contract_policy.assess_covariance(False, ())
        )
    )
    zeros = (0.0,) * vehicle_contract_policy.COVARIANCE_VALUES
    present = measurement_health_policy.assess_measurement_health(
        dataclasses.replace(
            _valid_sample(), covariance_present=True, covariance=zeros
        )
    )
    self.assertTrue(present.accepted)
    self.assertTrue(vehicle_contract_policy.is_all_zero_covariance(zeros))
    self.assertFalse(
        vehicle_contract_policy.covariance_is_unknown(
            vehicle_contract_policy.assess_covariance(True, zeros)
        )
    )
    short = measurement_health_policy.assess_measurement_health(
        dataclasses.replace(
            _valid_sample(), covariance_present=True, covariance=(0.0,) * 35
        )
    )
    self.assertIs(
        short.error, measurement_health_policy.MeasurementError.COVARIANCE
    )
    self.assertIs(
        short.state, measurement_health_policy.MeasurementStateKind.VALID
    )

  def test_empty_source_id_is_rejected_and_unset_validity_is_not(self):
    rejected = measurement_health_policy.assess_measurement_health(
        dataclasses.replace(
            _valid_sample(),
            sources=(
                measurement_health_policy.SourceHealthView("primary", True, 1),
                measurement_health_policy.SourceHealthView("", False, 0),
            ),
        )
    )
    self.assertIs(
        rejected.error, measurement_health_policy.MeasurementError.SOURCE_ID
    )
    aiding = measurement_health_policy.SourceHealthView("aiding", False, 0)
    accepted = measurement_health_policy.assess_measurement_health(
        dataclasses.replace(_valid_sample(), sources=(aiding,))
    )
    self.assertTrue(accepted.accepted)
    self.assertIs(
        stamped_header_policy.classify_validity(False, 0),
        stamped_header_policy.ValidityKind.ABSENT,
    )

  def test_first_defect_wins(self):
    missing = measurement_health_policy.assess_measurement_health(
        dataclasses.replace(_valid_sample(), frame_id="", quality=math.nan)
    )
    self.assertIs(
        missing.error, measurement_health_policy.MeasurementError.MISSING_FRAME
    )
    reversed_quality = measurement_health_policy.assess_measurement_health(
        dataclasses.replace(
            _valid_sample(),
            source_time=(2, 0),
            receive_time=(1, 0),
            quality=2.0,
        )
    )
    self.assertIs(
        reversed_quality.error,
        measurement_health_policy.MeasurementError.TIME_REVERSAL,
    )
    quality = measurement_health_policy.assess_measurement_health(
        dataclasses.replace(
            _valid_sample(),
            quality=2.0,
            covariance_present=True,
            covariance=(1.0,),
        )
    )
    self.assertIs(
        quality.error, measurement_health_policy.MeasurementError.QUALITY
    )


if __name__ == "__main__":
  unittest.main()
