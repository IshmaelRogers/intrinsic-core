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

"""Determinism and fault-injection tests for FakeSurfaceFix."""

import dataclasses
import unittest

from intrinsic.hardware.marine import fake_surface_fix
from intrinsic.hardware.marine import measurement_health_pb2
from intrinsic.hardware.marine import measurement_health_policy
from intrinsic.hardware.marine import surface_fix_pb2
from intrinsic.hardware.marine import surface_fix_policy

_ERROR = surface_fix_policy.SurfaceFixError
_STATE = measurement_health_pb2.MeasurementHealth


def _view_of(sample):
  health = sample.health
  covariance = (
      tuple(health.covariance.values) if health.HasField("covariance") else ()
  )
  sources = tuple(
      measurement_health_policy.SourceHealthView(
          source.source_id,
          source.HasField("validity"),
          source.validity.state,
      )
      for source in health.sources
  )
  header = health.header
  health_view = measurement_health_policy.MeasurementHealthView(
      header_present=sample.HasField("health"),
      frame_id=header.frame_id,
      source_time_present=header.HasField("source_time"),
      source_time=(header.source_time.seconds, header.source_time.nanos),
      receive_time_present=header.HasField("receive_time"),
      receive_time=(header.receive_time.seconds, header.receive_time.nanos),
      header_validity_present=header.HasField("validity"),
      header_validity_state=header.validity.state,
      state_present=health.HasField("state"),
      state=health.state,
      quality_present=health.HasField("quality"),
      quality=health.quality,
      covariance_present=health.HasField("covariance"),
      covariance=covariance,
      sources=sources,
  )
  return surface_fix_policy.SurfaceFixView(
      health=health_view,
      position_x_present=sample.HasField("position_x_m"),
      position_x_m=sample.position_x_m,
      position_y_present=sample.HasField("position_y_m"),
      position_y_m=sample.position_y_m,
      position_z_present=sample.HasField("position_z_m"),
      position_z_m=sample.position_z_m,
      source_present=sample.HasField("source"),
      source=sample.source,
      satellite_count_present=sample.HasField("satellite_count"),
      satellite_count=sample.satellite_count,
      beacon_count_present=sample.HasField("beacon_count"),
      beacon_count=sample.beacon_count,
      horizontal_accuracy_present=sample.HasField("horizontal_accuracy_m"),
      horizontal_accuracy_m=sample.horizontal_accuracy_m,
      vertical_accuracy_present=sample.HasField("vertical_accuracy_m"),
      vertical_accuracy_m=sample.vertical_accuracy_m,
      velocity_x_present=sample.HasField("velocity_x_m_s"),
      velocity_x_m_s=sample.velocity_x_m_s,
      velocity_y_present=sample.HasField("velocity_y_m_s"),
      velocity_y_m_s=sample.velocity_y_m_s,
      velocity_z_present=sample.HasField("velocity_z_m_s"),
      velocity_z_m_s=sample.velocity_z_m_s,
  )


def _assess(sample):
  return surface_fix_policy.assess_surface_fix(_view_of(sample))


class FakeSurfaceFixTest(unittest.TestCase):

  def test_fixed_seed_trace_is_repeatable(self):
    first = fake_surface_fix.FakeSurfaceFix()
    second = fake_surface_fix.FakeSurfaceFix(
        fake_surface_fix.FakeSurfaceFixConfig()
    )
    left = first.measure()
    right = second.measure()
    self.assertEqual(left.SerializeToString(), right.SerializeToString())
    self.assertEqual(
        first.measure().SerializeToString(), left.SerializeToString()
    )
    self.assertEqual(left.source, surface_fix_pb2.SurfaceFix.FIX_SOURCE_GNSS)
    self.assertEqual(left.health.header.frame_id, "gnss")
    self.assertEqual(left.position_x_m, 12.5)
    self.assertEqual(left.position_y_m, -3.25)
    self.assertEqual(left.position_z_m, 1.0)
    self.assertEqual(left.satellite_count, 12)
    self.assertEqual(left.health.quality, 0.75)
    self.assertEqual(left.health.state, _STATE.VALID)
    self.assertTrue(_assess(left).accepted)
    self.assertIsNone(
        surface_fix_pb2.SurfaceFix.DESCRIPTOR.fields_by_name.get(
            "orientation_xyzw"
        )
    )

  def test_seed_is_the_header_sequence(self):
    config = fake_surface_fix.FakeSurfaceFixConfig(seed=7)
    sample = fake_surface_fix.FakeSurfaceFix(config).measure()
    self.assertEqual(sample.health.header.sequence, 7)
    other = fake_surface_fix.FakeSurfaceFix(
        dataclasses.replace(config, seed=8)
    ).measure()
    self.assertNotEqual(other.SerializeToString(), sample.SerializeToString())

  def test_acoustic_truth_is_accepted(self):
    truth = fake_surface_fix.SurfaceFixTruth(
        frame_id="usbl",
        source=2,
        satellite_count_present=False,
        beacon_count_present=True,
        beacon_count=4,
    )
    sample = fake_surface_fix.FakeSurfaceFix().measure(truth)
    self.assertEqual(
        sample.source, surface_fix_pb2.SurfaceFix.FIX_SOURCE_ACOUSTIC
    )
    self.assertFalse(sample.HasField("satellite_count"))
    self.assertEqual(sample.beacon_count, 4)
    self.assertTrue(_assess(sample).accepted)

  def test_bias_is_added_degraded_and_not_repaired(self):
    config = fake_surface_fix.FakeSurfaceFixConfig(
        position_bias_x_m=0.5, position_bias_z_m=-2.0
    )
    sample = fake_surface_fix.FakeSurfaceFix(config).measure()
    self.assertEqual(sample.position_x_m, 13.0)
    self.assertEqual(sample.position_y_m, -3.25)
    self.assertEqual(sample.position_z_m, -1.0)
    self.assertEqual(sample.health.state, _STATE.DEGRADED)
    self.assertEqual(sample.health.header.validity.state, 1)
    assessment = _assess(sample)
    self.assertEqual(assessment.error, _ERROR.NONE)
    self.assertEqual(
        assessment.state,
        measurement_health_policy.MeasurementStateKind.DEGRADED,
    )
    self.assertFalse(assessment.accepted)

  def test_delay_sets_receive_time_and_reversal_is_not_repaired(self):
    config = fake_surface_fix.FakeSurfaceFixConfig(
        delay_seconds=2, delay_nanos=0
    )
    truth = fake_surface_fix.SurfaceFixTruth(source_seconds=100, source_nanos=0)
    delayed = fake_surface_fix.FakeSurfaceFix(config).measure(truth)
    self.assertEqual(delayed.health.header.receive_time.seconds, 102)
    self.assertEqual(delayed.health.header.receive_time.nanos, 0)
    self.assertEqual(delayed.health.state, _STATE.VALID)
    self.assertTrue(_assess(delayed).accepted)
    reversed_sample = fake_surface_fix.FakeSurfaceFix(
        dataclasses.replace(config, delay_seconds=-1)
    ).measure(truth)
    self.assertEqual(reversed_sample.health.header.source_time.seconds, 100)
    self.assertEqual(reversed_sample.health.header.receive_time.seconds, 99)
    self.assertEqual(_assess(reversed_sample).error, _ERROR.TIME_REVERSAL)

  def test_dropout_omits_the_sample(self):
    config = fake_surface_fix.FakeSurfaceFixConfig(
        dropout=True, invalid_fix=True, position_bias_x_m=1.0
    )
    self.assertIsNone(fake_surface_fix.FakeSurfaceFix(config).measure())
    absent = surface_fix_policy.assess_surface_fix(
        surface_fix_policy.SurfaceFixView()
    )
    self.assertEqual(absent.error, _ERROR.NONE)
    self.assertEqual(
        absent.state, measurement_health_policy.MeasurementStateKind.ABSENT
    )
    self.assertFalse(absent.accepted)

  def test_invalid_fix_is_canonical_unspecified_source_and_invalid(self):
    config = fake_surface_fix.FakeSurfaceFixConfig(
        invalid_fix=True, position_bias_x_m=4.0
    )
    sample = fake_surface_fix.FakeSurfaceFix(config).measure()
    self.assertTrue(sample.HasField("source"))
    self.assertEqual(
        sample.source, surface_fix_pb2.SurfaceFix.FIX_SOURCE_UNSPECIFIED
    )
    self.assertEqual(sample.health.state, _STATE.INVALID)
    self.assertEqual(sample.position_x_m, 12.5)
    assessment = _assess(sample)
    self.assertEqual(assessment.error, _ERROR.SOURCE)
    self.assertEqual(
        assessment.state, measurement_health_policy.MeasurementStateKind.INVALID
    )
    self.assertFalse(assessment.accepted)

  def test_invalid_quality_is_not_rewritten(self):
    sample = fake_surface_fix.FakeSurfaceFix().measure(
        fake_surface_fix.SurfaceFixTruth(quality=1.5)
    )
    self.assertEqual(sample.health.quality, 1.5)
    self.assertEqual(sample.health.state, _STATE.VALID)
    self.assertEqual(_assess(sample).error, _ERROR.QUALITY)
    self.assertEqual(sample.health.state, _STATE.VALID)

  def test_boundary_truths_are_accepted(self):
    truth = fake_surface_fix.SurfaceFixTruth(
        position_x_m=0.0,
        position_y_m=0.0,
        position_z_m=0.0,
        satellite_count=0,
        horizontal_accuracy_present=True,
        horizontal_accuracy_m=0.0,
        quality=0.0,
    )
    sample = fake_surface_fix.FakeSurfaceFix().measure(truth)
    self.assertTrue(sample.HasField("position_x_m"))
    self.assertTrue(sample.HasField("satellite_count"))
    self.assertTrue(sample.HasField("horizontal_accuracy_m"))
    self.assertTrue(_assess(sample).accepted)
    full = dataclasses.replace(truth, quality=1.0)
    self.assertTrue(
        _assess(fake_surface_fix.FakeSurfaceFix().measure(full)).accepted
    )

  def test_absent_covariance_and_off_slot_truths(self):
    sample = fake_surface_fix.FakeSurfaceFix().measure(
        fake_surface_fix.SurfaceFixTruth(covariance_present=False)
    )
    self.assertFalse(sample.health.HasField("covariance"))
    self.assertTrue(_assess(sample).accepted)
    sample = fake_surface_fix.FakeSurfaceFix().measure()
    sample.health.covariance.values[1] = 0.5
    sample.health.covariance.values[6] = 0.5
    self.assertEqual(_assess(sample).error, _ERROR.COVARIANCE_SLOTS)

  def test_missing_source_truth_is_rejected_and_stays_valid(self):
    sample = fake_surface_fix.FakeSurfaceFix().measure(
        fake_surface_fix.SurfaceFixTruth(source_present=False)
    )
    self.assertFalse(sample.HasField("source"))
    self.assertEqual(sample.health.state, _STATE.VALID)
    self.assertEqual(_assess(sample).error, _ERROR.SOURCE)

  def test_empty_frame_and_non_finite_position_are_rejected(self):
    empty = fake_surface_fix.FakeSurfaceFix().measure(
        fake_surface_fix.SurfaceFixTruth(frame_id="")
    )
    self.assertEqual(_assess(empty).error, _ERROR.MISSING_FRAME)
    nan = fake_surface_fix.FakeSurfaceFix().measure(
        fake_surface_fix.SurfaceFixTruth(position_y_m=float("nan"))
    )
    self.assertEqual(_assess(nan).error, _ERROR.NON_FINITE)


if __name__ == "__main__":
  unittest.main()
