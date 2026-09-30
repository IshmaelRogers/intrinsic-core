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

"""Validation tests for the altimeter measurement policy."""

import dataclasses
import math
import unittest

from intrinsic.hardware.marine import altimeter_policy
from intrinsic.hardware.marine import measurement_health_policy

from intrinsic.embodiment import frame_policy
from intrinsic.embodiment import stamped_header_policy
from intrinsic.vehicle import vehicle_contract_policy


def _covariance():
  values = [0.0] * vehicle_contract_policy.COVARIANCE_VALUES
  values[altimeter_policy.RANGE_VARIANCE_SLOT] = 0.25
  return tuple(values)


def _valid_range():
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
      covariance=_covariance(),
  )
  return altimeter_policy.AltimeterMeasurementView(
      health=health,
      range_present=True,
      range_m=10.0,
      beam_present=True,
      beam_id="down",
      min_present=True,
      min_range_m=0.5,
      max_present=True,
      max_range_m=100.0,
      has_return_present=True,
      has_return=True,
  )


def _health(**kwargs):
  return dataclasses.replace(_valid_range().health, **kwargs)


class AltimeterPolicyTest(unittest.TestCase):

  def test_empty_sample_is_absent_and_not_an_error(self):
    assessment = altimeter_policy.assess_altimeter(
        altimeter_policy.AltimeterMeasurementView()
    )
    self.assertEqual(assessment.error, altimeter_policy.AltimeterError.NONE)
    self.assertEqual(
        assessment.state, measurement_health_policy.MeasurementStateKind.ABSENT
    )
    self.assertEqual(
        assessment.header_validity, stamped_header_policy.ValidityKind.ABSENT
    )
    self.assertFalse(assessment.accepted)

  def test_nominal_range_is_accepted(self):
    assessment = altimeter_policy.assess_altimeter(_valid_range())
    self.assertEqual(assessment.error, altimeter_policy.AltimeterError.NONE)
    self.assertEqual(
        assessment.state, measurement_health_policy.MeasurementStateKind.VALID
    )
    self.assertEqual(
        assessment.header_validity, stamped_header_policy.ValidityKind.VALID
    )
    self.assertTrue(assessment.accepted)

  def test_range_only_and_return_flags(self):
    range_only = dataclasses.replace(
        _valid_range(), has_return_present=False, has_return=False
    )
    self.assertTrue(altimeter_policy.assess_altimeter(range_only).accepted)
    self.assertFalse(altimeter_policy.explicit_no_return(range_only))
    true_without_range = dataclasses.replace(
        _valid_range(), range_present=False, range_m=0.0
    )
    self.assertEqual(
        altimeter_policy.assess_altimeter(true_without_range).error,
        altimeter_policy.AltimeterError.PAYLOAD,
    )

  def test_range_endpoints_and_sign(self):
    contact = dataclasses.replace(_valid_range(), range_m=0.0, min_range_m=0.0)
    self.assertTrue(altimeter_policy.assess_altimeter(contact).accepted)
    at_min = dataclasses.replace(_valid_range(), range_m=0.5)
    self.assertTrue(altimeter_policy.assess_altimeter(at_min).accepted)
    at_max = dataclasses.replace(_valid_range(), range_m=100.0)
    self.assertTrue(altimeter_policy.assess_altimeter(at_max).accepted)
    equal_bounds = dataclasses.replace(
        _valid_range(), min_range_m=10.0, max_range_m=10.0, range_m=10.0
    )
    self.assertTrue(altimeter_policy.assess_altimeter(equal_bounds).accepted)
    negative = dataclasses.replace(
        _valid_range(), range_m=math.nextafter(0.0, -1.0)
    )
    assessment = altimeter_policy.assess_altimeter(negative)
    self.assertEqual(assessment.error, altimeter_policy.AltimeterError.RANGE)
    self.assertEqual(
        assessment.state, measurement_health_policy.MeasurementStateKind.VALID
    )
    self.assertEqual(negative.health.state, 1)
    nan = dataclasses.replace(_valid_range(), range_m=math.nan)
    self.assertEqual(
        altimeter_policy.assess_altimeter(nan).error,
        altimeter_policy.AltimeterError.NON_FINITE,
    )

  def test_bounds_and_out_of_range(self):
    below = dataclasses.replace(
        _valid_range(), range_m=math.nextafter(0.5, 0.0)
    )
    self.assertEqual(
        altimeter_policy.assess_altimeter(below).error,
        altimeter_policy.AltimeterError.BOUNDS,
    )
    above = dataclasses.replace(
        _valid_range(), range_m=math.nextafter(100.0, 200.0)
    )
    self.assertEqual(
        altimeter_policy.assess_altimeter(above).error,
        altimeter_policy.AltimeterError.BOUNDS,
    )
    self.assertEqual(above.health.state, 1)
    swapped = dataclasses.replace(
        _valid_range(), min_range_m=20.0, max_range_m=10.0, range_m=15.0
    )
    self.assertEqual(
        altimeter_policy.assess_altimeter(swapped).error,
        altimeter_policy.AltimeterError.BOUNDS,
    )
    negative_min = dataclasses.replace(_valid_range(), min_range_m=-0.1)
    self.assertEqual(
        altimeter_policy.assess_altimeter(negative_min).error,
        altimeter_policy.AltimeterError.BOUNDS,
    )
    nan_max = dataclasses.replace(_valid_range(), max_range_m=math.inf)
    self.assertEqual(
        altimeter_policy.assess_altimeter(nan_max).error,
        altimeter_policy.AltimeterError.NON_FINITE,
    )

  def test_no_return_consistency(self):
    no_return = dataclasses.replace(
        _valid_range(),
        range_present=False,
        range_m=0.0,
        has_return=False,
        health=_health(state=3),
    )
    consistent = altimeter_policy.assess_altimeter(no_return)
    self.assertEqual(consistent.error, altimeter_policy.AltimeterError.NONE)
    self.assertEqual(
        consistent.state, measurement_health_policy.MeasurementStateKind.INVALID
    )
    self.assertFalse(consistent.accepted)
    self.assertEqual(no_return.health.state, 3)
    valid_loss = dataclasses.replace(no_return, health=_health(state=1))
    self.assertEqual(
        altimeter_policy.assess_altimeter(valid_loss).error,
        altimeter_policy.AltimeterError.NO_RETURN,
    )
    with_range = dataclasses.replace(
        _valid_range(), has_return=False, health=_health(state=3)
    )
    self.assertEqual(
        altimeter_policy.assess_altimeter(with_range).error,
        altimeter_policy.AltimeterError.NO_RETURN,
    )

  def test_beam_id_presence(self):
    absent = dataclasses.replace(_valid_range(), beam_present=False, beam_id="")
    self.assertTrue(altimeter_policy.assess_altimeter(absent).accepted)
    empty = dataclasses.replace(_valid_range(), beam_id="")
    self.assertEqual(
        altimeter_policy.assess_altimeter(empty).error,
        altimeter_policy.AltimeterError.BEAM_ID,
    )

  def test_quality_endpoints(self):
    low = dataclasses.replace(_valid_range(), health=_health(quality=0.0))
    self.assertTrue(altimeter_policy.assess_altimeter(low).accepted)
    high = dataclasses.replace(_valid_range(), health=_health(quality=1.0))
    self.assertTrue(altimeter_policy.assess_altimeter(high).accepted)
    above = dataclasses.replace(
        _valid_range(), health=_health(quality=math.nextafter(1.0, 2.0))
    )
    self.assertEqual(
        altimeter_policy.assess_altimeter(above).error,
        altimeter_policy.AltimeterError.QUALITY,
    )

  def test_covariance_absence_zero_and_slots(self):
    absent = dataclasses.replace(
        _valid_range(),
        health=_health(covariance_present=False, covariance=()),
    )
    self.assertTrue(altimeter_policy.assess_altimeter(absent).accepted)
    zeros = tuple([0.0] * vehicle_contract_policy.COVARIANCE_VALUES)
    zero = dataclasses.replace(_valid_range(), health=_health(covariance=zeros))
    self.assertTrue(altimeter_policy.assess_altimeter(zero).accepted)
    self.assertTrue(vehicle_contract_policy.is_all_zero_covariance(zeros))
    short = dataclasses.replace(
        _valid_range(), health=_health(covariance=tuple([0.0] * 35))
    )
    self.assertEqual(
        altimeter_policy.assess_altimeter(short).error,
        altimeter_policy.AltimeterError.COVARIANCE,
    )
    off = list(_covariance())
    off[vehicle_contract_policy.covariance_index(1, 1)] = 0.25
    slots = dataclasses.replace(
        _valid_range(), health=_health(covariance=tuple(off))
    )
    self.assertEqual(
        altimeter_policy.assess_altimeter(slots).error,
        altimeter_policy.AltimeterError.COVARIANCE_SLOTS,
    )

  def test_metadata_defects_use_health_order(self):
    missing_frame = dataclasses.replace(
        _valid_range(), health=_health(frame_id="")
    )
    self.assertEqual(
        altimeter_policy.assess_altimeter(missing_frame).error,
        altimeter_policy.AltimeterError.MISSING_FRAME,
    )
    wrong = dataclasses.replace(
        _valid_range(),
        health=_health(
            expected_frame_present=True,
            expected_frame=frame_policy.WORLD_ENU_FRAME_ID,
        ),
    )
    self.assertEqual(
        altimeter_policy.assess_altimeter(wrong).error,
        altimeter_policy.AltimeterError.WRONG_FRAME,
    )
    reversed_time = dataclasses.replace(
        _valid_range(),
        health=_health(receive_time=(1700000000, 249999999)),
    )
    self.assertEqual(
        altimeter_policy.assess_altimeter(reversed_time).error,
        altimeter_policy.AltimeterError.TIME_REVERSAL,
    )
    delayed = dataclasses.replace(
        _valid_range(), health=_health(receive_time=(1700000100, 0))
    )
    self.assertTrue(altimeter_policy.assess_altimeter(delayed).accepted)
    empty_source = measurement_health_policy.SourceHealthView("", False, 0)
    source = dataclasses.replace(
        _valid_range(), health=_health(sources=(empty_source,))
    )
    self.assertEqual(
        altimeter_policy.assess_altimeter(source).error,
        altimeter_policy.AltimeterError.SOURCE_ID,
    )

  def test_missing_health_and_validity_stay_put(self):
    payload_only = altimeter_policy.AltimeterMeasurementView(
        range_present=True, range_m=1.0
    )
    self.assertEqual(
        altimeter_policy.assess_altimeter(payload_only).error,
        altimeter_policy.AltimeterError.MISSING_HEALTH,
    )
    sample = dataclasses.replace(
        _valid_range(), health=_health(header_validity_state=2)
    )
    invalid_header = altimeter_policy.assess_altimeter(sample)
    self.assertTrue(invalid_header.accepted)
    self.assertEqual(
        invalid_header.header_validity,
        stamped_header_policy.ValidityKind.INVALID,
    )
    degraded = dataclasses.replace(
        _valid_range(),
        health=_health(state=2, header_validity_present=False),
    )
    assessment = altimeter_policy.assess_altimeter(degraded)
    self.assertEqual(assessment.error, altimeter_policy.AltimeterError.NONE)
    self.assertEqual(
        assessment.state,
        measurement_health_policy.MeasurementStateKind.DEGRADED,
    )
    self.assertFalse(assessment.accepted)
    self.assertEqual(degraded.health.state, 2)

  def test_first_defect_wins(self):
    missing_frame = dataclasses.replace(
        _valid_range(), range_m=-1.0, health=_health(frame_id="")
    )
    self.assertEqual(
        altimeter_policy.assess_altimeter(missing_frame).error,
        altimeter_policy.AltimeterError.MISSING_FRAME,
    )
    quality = dataclasses.replace(
        _valid_range(), range_m=-1.0, health=_health(quality=2.0)
    )
    self.assertEqual(
        altimeter_policy.assess_altimeter(quality).error,
        altimeter_policy.AltimeterError.QUALITY,
    )
    payload = dataclasses.replace(
        _valid_range(),
        range_present=False,
        has_return_present=False,
        beam_id="",
    )
    self.assertEqual(
        altimeter_policy.assess_altimeter(payload).error,
        altimeter_policy.AltimeterError.PAYLOAD,
    )
    outside = dataclasses.replace(
        _valid_range(), range_m=200.0, has_return=False, health=_health(state=3)
    )
    self.assertEqual(
        altimeter_policy.assess_altimeter(outside).error,
        altimeter_policy.AltimeterError.BOUNDS,
    )


if __name__ == "__main__":
  unittest.main()
