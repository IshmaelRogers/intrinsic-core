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

"""Validation tests for the pressure and depth measurement policy."""

import dataclasses
import math
import unittest

from intrinsic.hardware.marine import measurement_health_policy
from intrinsic.hardware.marine import pressure_depth_policy

from intrinsic.embodiment import frame_policy
from intrinsic.embodiment import stamped_header_policy
from intrinsic.vehicle import vehicle_contract_policy


def _covariance():
  values = [0.0] * vehicle_contract_policy.COVARIANCE_VALUES
  values[pressure_depth_policy.PRESSURE_VARIANCE_SLOT] = 1.0
  values[pressure_depth_policy.DEPTH_VARIANCE_SLOT] = 0.25
  return tuple(values)


def _valid_from_pressure():
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
  return pressure_depth_policy.PressureDepthMeasurementView(
      health=health,
      pressure_present=True,
      pressure_pa=200000.0,
      depth_present=True,
      depth_m=10.0,
      provenance_present=True,
      depth_provenance=2,
      density_present=True,
      fluid_density_kg_m3=1025.0,
  )


class PressureDepthPolicyTest(unittest.TestCase):

  def test_empty_sample_is_absent_and_not_an_error(self):
    assessment = pressure_depth_policy.assess_pressure_depth(
        pressure_depth_policy.PressureDepthMeasurementView()
    )
    self.assertEqual(
        assessment.error, pressure_depth_policy.PressureDepthError.NONE
    )
    self.assertEqual(
        assessment.state, measurement_health_policy.MeasurementStateKind.ABSENT
    )
    self.assertEqual(
        assessment.provenance, pressure_depth_policy.DepthProvenanceKind.ABSENT
    )
    self.assertEqual(
        assessment.header_validity, stamped_header_policy.ValidityKind.ABSENT
    )
    self.assertFalse(assessment.accepted)

  def test_nominal_from_pressure_is_accepted(self):
    assessment = pressure_depth_policy.assess_pressure_depth(
        _valid_from_pressure()
    )
    self.assertEqual(
        assessment.error, pressure_depth_policy.PressureDepthError.NONE
    )
    self.assertEqual(
        assessment.state, measurement_health_policy.MeasurementStateKind.VALID
    )
    self.assertEqual(
        assessment.provenance,
        pressure_depth_policy.DepthProvenanceKind.FROM_PRESSURE,
    )
    self.assertEqual(
        assessment.header_validity, stamped_header_policy.ValidityKind.VALID
    )
    self.assertTrue(assessment.accepted)

  def test_pressure_only_and_direct_depth_are_accepted(self):
    pressure_only = dataclasses.replace(
        _valid_from_pressure(),
        depth_present=False,
        depth_m=0.0,
        provenance_present=False,
        depth_provenance=0,
        density_present=False,
        fluid_density_kg_m3=0.0,
    )
    self.assertTrue(
        pressure_depth_policy.assess_pressure_depth(pressure_only).accepted
    )
    direct = dataclasses.replace(
        _valid_from_pressure(),
        pressure_present=False,
        pressure_pa=0.0,
        depth_provenance=1,
        density_present=False,
    )
    assessment = pressure_depth_policy.assess_pressure_depth(direct)
    self.assertTrue(assessment.accepted)
    self.assertEqual(
        assessment.provenance, pressure_depth_policy.DepthProvenanceKind.DIRECT
    )

  def test_provenance_absence_is_distinct_from_unspecified(self):
    classify = pressure_depth_policy.classify_depth_provenance
    kind = pressure_depth_policy.DepthProvenanceKind
    self.assertEqual(classify(False, 0), kind.ABSENT)
    self.assertEqual(classify(True, 0), kind.UNSPECIFIED)
    self.assertEqual(classify(True, 1), kind.DIRECT)
    self.assertEqual(classify(True, 2), kind.FROM_PRESSURE)
    self.assertEqual(classify(True, 100), kind.UNRECOGNIZED)
    self.assertNotEqual(classify(True, 100), kind.UNSPECIFIED)

    absent = dataclasses.replace(
        _valid_from_pressure(), provenance_present=False, depth_provenance=0
    )
    self.assertEqual(
        pressure_depth_policy.assess_pressure_depth(absent).error,
        pressure_depth_policy.PressureDepthError.PROVENANCE,
    )
    unspecified = dataclasses.replace(
        _valid_from_pressure(), depth_provenance=0
    )
    self.assertEqual(
        pressure_depth_policy.assess_pressure_depth(unspecified).provenance,
        kind.UNSPECIFIED,
    )
    unknown = dataclasses.replace(_valid_from_pressure(), depth_provenance=100)
    assessment = pressure_depth_policy.assess_pressure_depth(unknown)
    self.assertEqual(
        assessment.error, pressure_depth_policy.PressureDepthError.PROVENANCE
    )
    self.assertEqual(assessment.provenance, kind.UNRECOGNIZED)
    self.assertEqual(unknown.depth_provenance, 100)

  def test_pressure_bounds_are_finite_only(self):
    self.assertTrue(
        pressure_depth_policy.assess_pressure_depth(
            dataclasses.replace(_valid_from_pressure(), pressure_pa=0.0)
        ).accepted
    )
    self.assertTrue(
        pressure_depth_policy.assess_pressure_depth(
            dataclasses.replace(_valid_from_pressure(), pressure_pa=-101325.0)
        ).accepted
    )
    self.assertTrue(
        pressure_depth_policy.assess_pressure_depth(
            dataclasses.replace(_valid_from_pressure(), pressure_pa=1.0e12)
        ).accepted
    )
    nan = dataclasses.replace(_valid_from_pressure(), pressure_pa=math.nan)
    assessment = pressure_depth_policy.assess_pressure_depth(nan)
    self.assertEqual(
        assessment.error, pressure_depth_policy.PressureDepthError.NON_FINITE
    )
    self.assertEqual(
        assessment.state, measurement_health_policy.MeasurementStateKind.VALID
    )
    self.assertEqual(nan.health.state, 1)
    infinite = dataclasses.replace(_valid_from_pressure(), pressure_pa=math.inf)
    self.assertEqual(
        pressure_depth_policy.assess_pressure_depth(infinite).error,
        pressure_depth_policy.PressureDepthError.NON_FINITE,
    )

  def test_depth_surface_zero_and_negative(self):
    self.assertTrue(
        pressure_depth_policy.assess_pressure_depth(
            dataclasses.replace(_valid_from_pressure(), depth_m=0.0)
        ).accepted
    )
    negative = dataclasses.replace(
        _valid_from_pressure(), depth_m=math.nextafter(0.0, -1.0)
    )
    self.assertEqual(
        pressure_depth_policy.assess_pressure_depth(negative).error,
        pressure_depth_policy.PressureDepthError.DEPTH,
    )
    self.assertEqual(negative.health.state, 1)
    nan = dataclasses.replace(_valid_from_pressure(), depth_m=math.nan)
    self.assertEqual(
        pressure_depth_policy.assess_pressure_depth(nan).error,
        pressure_depth_policy.PressureDepthError.NON_FINITE,
    )

  def test_density_and_from_pressure_rules(self):
    tiny = dataclasses.replace(
        _valid_from_pressure(),
        fluid_density_kg_m3=math.nextafter(0.0, 1.0),
    )
    self.assertTrue(pressure_depth_policy.assess_pressure_depth(tiny).accepted)
    for density in (0.0, -1.0, math.nan):
      sample = dataclasses.replace(
          _valid_from_pressure(), fluid_density_kg_m3=density
      )
      self.assertEqual(
          pressure_depth_policy.assess_pressure_depth(sample).error,
          pressure_depth_policy.PressureDepthError.DENSITY,
      )
    missing_density = dataclasses.replace(
        _valid_from_pressure(), density_present=False
    )
    self.assertEqual(
        pressure_depth_policy.assess_pressure_depth(missing_density).error,
        pressure_depth_policy.PressureDepthError.DENSITY,
    )
    missing_pressure = dataclasses.replace(
        _valid_from_pressure(), pressure_present=False
    )
    self.assertEqual(
        pressure_depth_policy.assess_pressure_depth(missing_pressure).error,
        pressure_depth_policy.PressureDepthError.PROVENANCE,
    )
    no_depth = dataclasses.replace(_valid_from_pressure(), depth_present=False)
    self.assertEqual(
        pressure_depth_policy.assess_pressure_depth(no_depth).error,
        pressure_depth_policy.PressureDepthError.PROVENANCE,
    )
    direct = dataclasses.replace(
        _valid_from_pressure(), depth_provenance=1, density_present=False
    )
    self.assertTrue(
        pressure_depth_policy.assess_pressure_depth(direct).accepted
    )
    direct_zero = dataclasses.replace(
        direct, density_present=True, fluid_density_kg_m3=0.0
    )
    self.assertEqual(
        pressure_depth_policy.assess_pressure_depth(direct_zero).error,
        pressure_depth_policy.PressureDepthError.DENSITY,
    )
    pressure_unspecified = dataclasses.replace(
        _valid_from_pressure(),
        depth_present=False,
        depth_provenance=0,
        density_present=False,
    )
    self.assertTrue(
        pressure_depth_policy.assess_pressure_depth(
            pressure_unspecified
        ).accepted
    )
    pressure_unknown = dataclasses.replace(
        pressure_unspecified, depth_provenance=100
    )
    assessment = pressure_depth_policy.assess_pressure_depth(pressure_unknown)
    self.assertTrue(assessment.accepted)
    self.assertEqual(
        assessment.provenance,
        pressure_depth_policy.DepthProvenanceKind.UNRECOGNIZED,
    )

  def test_quality_endpoints_and_neither_payload(self):
    low = dataclasses.replace(
        _valid_from_pressure(),
        health=dataclasses.replace(_valid_from_pressure().health, quality=0.0),
    )
    high = dataclasses.replace(
        _valid_from_pressure(),
        health=dataclasses.replace(_valid_from_pressure().health, quality=1.0),
    )
    self.assertTrue(pressure_depth_policy.assess_pressure_depth(low).accepted)
    self.assertTrue(pressure_depth_policy.assess_pressure_depth(high).accepted)
    above = dataclasses.replace(
        _valid_from_pressure(),
        health=dataclasses.replace(
            _valid_from_pressure().health, quality=math.nextafter(1.0, 2.0)
        ),
    )
    self.assertEqual(
        pressure_depth_policy.assess_pressure_depth(above).error,
        pressure_depth_policy.PressureDepthError.QUALITY,
    )
    provenance_only = dataclasses.replace(
        _valid_from_pressure(),
        pressure_present=False,
        depth_present=False,
        pressure_pa=math.nan,
    )
    self.assertEqual(
        pressure_depth_policy.assess_pressure_depth(provenance_only).error,
        pressure_depth_policy.PressureDepthError.PAYLOAD,
    )
    invalid = dataclasses.replace(
        _valid_from_pressure(),
        pressure_pa=-5.0,
        health=dataclasses.replace(_valid_from_pressure().health, state=3),
    )
    marked = pressure_depth_policy.assess_pressure_depth(invalid)
    self.assertEqual(
        marked.error, pressure_depth_policy.PressureDepthError.NONE
    )
    self.assertEqual(
        marked.state, measurement_health_policy.MeasurementStateKind.INVALID
    )
    self.assertFalse(marked.accepted)
    self.assertEqual(invalid.health.state, 3)

  def test_covariance_absence_zero_and_slots(self):
    absent = dataclasses.replace(
        _valid_from_pressure(),
        health=dataclasses.replace(
            _valid_from_pressure().health,
            covariance_present=False,
            covariance=(),
        ),
    )
    self.assertTrue(
        pressure_depth_policy.assess_pressure_depth(absent).accepted
    )
    zeros = tuple([0.0] * vehicle_contract_policy.COVARIANCE_VALUES)
    zero = dataclasses.replace(
        _valid_from_pressure(),
        health=dataclasses.replace(
            _valid_from_pressure().health, covariance=zeros
        ),
    )
    self.assertTrue(pressure_depth_policy.assess_pressure_depth(zero).accepted)
    self.assertTrue(vehicle_contract_policy.is_all_zero_covariance(zeros))
    short = dataclasses.replace(
        _valid_from_pressure(),
        health=dataclasses.replace(
            _valid_from_pressure().health, covariance=tuple([0.0] * 35)
        ),
    )
    self.assertEqual(
        pressure_depth_policy.assess_pressure_depth(short).error,
        pressure_depth_policy.PressureDepthError.COVARIANCE,
    )
    off = list(_covariance())
    off[vehicle_contract_policy.covariance_index(2, 2)] = 0.25
    slots = dataclasses.replace(
        _valid_from_pressure(),
        health=dataclasses.replace(
            _valid_from_pressure().health, covariance=tuple(off)
        ),
    )
    self.assertEqual(
        pressure_depth_policy.assess_pressure_depth(slots).error,
        pressure_depth_policy.PressureDepthError.COVARIANCE_SLOTS,
    )
    cross = list(_covariance())
    cross[vehicle_contract_policy.covariance_index(0, 1)] = 0.1
    cross[vehicle_contract_policy.covariance_index(1, 0)] = 0.1
    symmetric = dataclasses.replace(
        _valid_from_pressure(),
        health=dataclasses.replace(
            _valid_from_pressure().health, covariance=tuple(cross)
        ),
    )
    self.assertEqual(
        pressure_depth_policy.assess_pressure_depth(symmetric).error,
        pressure_depth_policy.PressureDepthError.COVARIANCE_SLOTS,
    )

  def test_metadata_defects_use_health_order(self):
    missing_frame = dataclasses.replace(
        _valid_from_pressure(),
        health=dataclasses.replace(_valid_from_pressure().health, frame_id=""),
    )
    self.assertEqual(
        pressure_depth_policy.assess_pressure_depth(missing_frame).error,
        pressure_depth_policy.PressureDepthError.MISSING_FRAME,
    )
    wrong = dataclasses.replace(
        _valid_from_pressure(),
        health=dataclasses.replace(
            _valid_from_pressure().health,
            expected_frame_present=True,
            expected_frame=frame_policy.WORLD_ENU_FRAME_ID,
        ),
    )
    self.assertEqual(
        pressure_depth_policy.assess_pressure_depth(wrong).error,
        pressure_depth_policy.PressureDepthError.WRONG_FRAME,
    )
    matched = dataclasses.replace(
        wrong,
        health=dataclasses.replace(
            wrong.health, frame_id=frame_policy.WORLD_ENU_FRAME_ID
        ),
    )
    self.assertTrue(
        pressure_depth_policy.assess_pressure_depth(matched).accepted
    )
    reversed_time = dataclasses.replace(
        _valid_from_pressure(),
        health=dataclasses.replace(
            _valid_from_pressure().health, receive_time=(1700000000, 249999999)
        ),
    )
    self.assertEqual(
        pressure_depth_policy.assess_pressure_depth(reversed_time).error,
        pressure_depth_policy.PressureDepthError.TIME_REVERSAL,
    )
    delayed = dataclasses.replace(
        _valid_from_pressure(),
        health=dataclasses.replace(
            _valid_from_pressure().health, receive_time=(1700000100, 0)
        ),
    )
    self.assertTrue(
        pressure_depth_policy.assess_pressure_depth(delayed).accepted
    )
    quality_nan = dataclasses.replace(
        _valid_from_pressure(),
        health=dataclasses.replace(
            _valid_from_pressure().health, quality=math.nan
        ),
    )
    assessment = pressure_depth_policy.assess_pressure_depth(quality_nan)
    self.assertEqual(
        assessment.error, pressure_depth_policy.PressureDepthError.NON_FINITE
    )
    self.assertEqual(
        assessment.state, measurement_health_policy.MeasurementStateKind.VALID
    )
    source = dataclasses.replace(
        _valid_from_pressure(),
        health=dataclasses.replace(
            _valid_from_pressure().health,
            sources=(measurement_health_policy.SourceHealthView(""),),
        ),
    )
    self.assertEqual(
        pressure_depth_policy.assess_pressure_depth(source).error,
        pressure_depth_policy.PressureDepthError.SOURCE_ID,
    )

  def test_missing_health_precedes_payload_and_validity_stays_put(self):
    payload_only = pressure_depth_policy.PressureDepthMeasurementView(
        pressure_present=True, pressure_pa=1.0
    )
    self.assertEqual(
        pressure_depth_policy.assess_pressure_depth(payload_only).error,
        pressure_depth_policy.PressureDepthError.MISSING_HEALTH,
    )
    invalid_header = dataclasses.replace(
        _valid_from_pressure(),
        health=dataclasses.replace(
            _valid_from_pressure().health, header_validity_state=2
        ),
    )
    header = pressure_depth_policy.assess_pressure_depth(invalid_header)
    self.assertTrue(header.accepted)
    self.assertEqual(
        header.header_validity, stamped_header_policy.ValidityKind.INVALID
    )
    degraded = dataclasses.replace(
        _valid_from_pressure(),
        health=dataclasses.replace(
            _valid_from_pressure().health,
            state=2,
            header_validity_present=False,
        ),
    )
    assessment = pressure_depth_policy.assess_pressure_depth(degraded)
    self.assertEqual(
        assessment.error, pressure_depth_policy.PressureDepthError.NONE
    )
    self.assertEqual(
        assessment.state,
        measurement_health_policy.MeasurementStateKind.DEGRADED,
    )
    self.assertFalse(assessment.accepted)
    self.assertEqual(degraded.health.state, 2)
    self.assertEqual(
        stamped_header_policy.classify_validity(True, 3),
        stamped_header_policy.ValidityKind.UNSPECIFIED,
    )

  def test_first_defect_wins(self):
    missing_frame = dataclasses.replace(
        _valid_from_pressure(),
        depth_m=-1.0,
        health=dataclasses.replace(_valid_from_pressure().health, frame_id=""),
    )
    self.assertEqual(
        pressure_depth_policy.assess_pressure_depth(missing_frame).error,
        pressure_depth_policy.PressureDepthError.MISSING_FRAME,
    )
    quality = dataclasses.replace(
        _valid_from_pressure(),
        depth_provenance=0,
        health=dataclasses.replace(_valid_from_pressure().health, quality=2.0),
    )
    self.assertEqual(
        pressure_depth_policy.assess_pressure_depth(quality).error,
        pressure_depth_policy.PressureDepthError.QUALITY,
    )
    slotted = list(_covariance())
    slotted[vehicle_contract_policy.covariance_index(2, 2)] = 1.0
    slots = dataclasses.replace(
        _valid_from_pressure(),
        depth_m=-1.0,
        health=dataclasses.replace(
            _valid_from_pressure().health, covariance=tuple(slotted)
        ),
    )
    self.assertEqual(
        pressure_depth_policy.assess_pressure_depth(slots).error,
        pressure_depth_policy.PressureDepthError.COVARIANCE_SLOTS,
    )
    payload = dataclasses.replace(
        _valid_from_pressure(),
        pressure_present=False,
        depth_present=False,
        depth_m=-1.0,
    )
    self.assertEqual(
        pressure_depth_policy.assess_pressure_depth(payload).error,
        pressure_depth_policy.PressureDepthError.PAYLOAD,
    )
    nan = dataclasses.replace(
        _valid_from_pressure(), pressure_pa=math.nan, depth_m=-1.0
    )
    self.assertEqual(
        pressure_depth_policy.assess_pressure_depth(nan).error,
        pressure_depth_policy.PressureDepthError.NON_FINITE,
    )
    depth = dataclasses.replace(
        _valid_from_pressure(), depth_m=-1.0, depth_provenance=0
    )
    self.assertEqual(
        pressure_depth_policy.assess_pressure_depth(depth).error,
        pressure_depth_policy.PressureDepthError.DEPTH,
    )
    provenance = dataclasses.replace(
        _valid_from_pressure(), depth_provenance=0, fluid_density_kg_m3=0.0
    )
    self.assertEqual(
        pressure_depth_policy.assess_pressure_depth(provenance).error,
        pressure_depth_policy.PressureDepthError.PROVENANCE,
    )
    missing_pressure = dataclasses.replace(
        _valid_from_pressure(), pressure_present=False, fluid_density_kg_m3=-1.0
    )
    self.assertEqual(
        pressure_depth_policy.assess_pressure_depth(missing_pressure).error,
        pressure_depth_policy.PressureDepthError.PROVENANCE,
    )
    density = dataclasses.replace(_valid_from_pressure(), density_present=False)
    self.assertEqual(
        pressure_depth_policy.assess_pressure_depth(density).error,
        pressure_depth_policy.PressureDepthError.DENSITY,
    )


if __name__ == "__main__":
  unittest.main()
