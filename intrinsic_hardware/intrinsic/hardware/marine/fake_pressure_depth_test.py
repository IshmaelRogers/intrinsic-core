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

"""Determinism and fault-injection tests for FakePressureDepth."""

import dataclasses
import unittest

from intrinsic.hardware.marine import fake_pressure_depth
from intrinsic.hardware.marine import measurement_health_pb2
from intrinsic.hardware.marine import measurement_health_policy
from intrinsic.hardware.marine import pressure_depth_pb2
from intrinsic.hardware.marine import pressure_depth_policy


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
  return pressure_depth_policy.PressureDepthMeasurementView(
      health=health_view,
      pressure_present=sample.HasField("pressure_pa"),
      pressure_pa=sample.pressure_pa,
      depth_present=sample.HasField("depth_m"),
      depth_m=sample.depth_m,
      provenance_present=sample.HasField("depth_provenance"),
      depth_provenance=sample.depth_provenance,
      density_present=sample.HasField("fluid_density_kg_m3"),
      fluid_density_kg_m3=sample.fluid_density_kg_m3,
  )


def _assess(sample):
  return pressure_depth_policy.assess_pressure_depth(_view_of(sample))


class FakePressureDepthTest(unittest.TestCase):

  def test_fixed_seed_trace_is_repeatable(self):
    first = fake_pressure_depth.FakePressureDepth()
    second = fake_pressure_depth.FakePressureDepth(
        fake_pressure_depth.FakePressureDepthConfig()
    )
    left = first.measure()
    right = second.measure()
    self.assertEqual(left.SerializeToString(), right.SerializeToString())
    self.assertEqual(
        first.measure().SerializeToString(), left.SerializeToString()
    )
    self.assertTrue(_assess(left).accepted)
    self.assertIsNone(
        pressure_depth_pb2.PressureDepthMeasurement.DESCRIPTOR.fields_by_name.get(
            "altitude_m"
        )
    )

  def test_seed_is_the_header_sequence(self):
    config = fake_pressure_depth.FakePressureDepthConfig(seed=7)
    sample = fake_pressure_depth.FakePressureDepth(config).measure()
    self.assertEqual(sample.health.header.sequence, 7)
    other = fake_pressure_depth.FakePressureDepth(
        dataclasses.replace(config, seed=8)
    )
    self.assertNotEqual(
        other.measure().SerializeToString(), sample.SerializeToString()
    )

  def test_bias_is_degraded_and_added(self):
    config = fake_pressure_depth.FakePressureDepthConfig(
        pressure_bias_pa=100.0, depth_bias_m=0.5
    )
    sample = fake_pressure_depth.FakePressureDepth(config).measure()
    self.assertEqual(sample.pressure_pa, 200100.0)
    self.assertEqual(sample.depth_m, 10.5)
    self.assertEqual(
        sample.health.state, measurement_health_pb2.MeasurementHealth.DEGRADED
    )
    self.assertEqual(sample.health.header.validity.state, 1)
    assessment = _assess(sample)
    self.assertEqual(
        assessment.error, pressure_depth_policy.PressureDepthError.NONE
    )
    self.assertEqual(
        assessment.state,
        measurement_health_policy.MeasurementStateKind.DEGRADED,
    )
    self.assertFalse(assessment.accepted)
    self.assertEqual(
        sample.health.state, measurement_health_pb2.MeasurementHealth.DEGRADED
    )

  def test_depth_bias_below_the_surface_is_not_rewritten(self):
    config = fake_pressure_depth.FakePressureDepthConfig(depth_bias_m=-11.0)
    sample = fake_pressure_depth.FakePressureDepth(config).measure()
    self.assertEqual(sample.depth_m, -1.0)
    self.assertEqual(
        sample.health.state, measurement_health_pb2.MeasurementHealth.DEGRADED
    )
    assessment = _assess(sample)
    self.assertEqual(
        assessment.error, pressure_depth_policy.PressureDepthError.DEPTH
    )
    self.assertEqual(
        assessment.state,
        measurement_health_policy.MeasurementStateKind.DEGRADED,
    )
    self.assertEqual(
        sample.health.state, measurement_health_pb2.MeasurementHealth.DEGRADED
    )

  def test_delay_sets_receive_time_and_reversal_is_not_repaired(self):
    config = fake_pressure_depth.FakePressureDepthConfig(
        delay_seconds=2, delay_nanos=0
    )
    truth = fake_pressure_depth.PressureDepthTruth(
        source_seconds=100, source_nanos=0
    )
    delayed = fake_pressure_depth.FakePressureDepth(config).measure(truth)
    self.assertEqual(delayed.health.header.receive_time.seconds, 102)
    self.assertEqual(delayed.health.header.receive_time.nanos, 0)
    self.assertEqual(
        delayed.health.state, measurement_health_pb2.MeasurementHealth.VALID
    )
    self.assertTrue(_assess(delayed).accepted)
    reversed_config = dataclasses.replace(config, delay_seconds=-1)
    reversed_sample = fake_pressure_depth.FakePressureDepth(
        reversed_config
    ).measure(truth)
    self.assertEqual(reversed_sample.health.header.source_time.seconds, 100)
    self.assertEqual(reversed_sample.health.header.receive_time.seconds, 99)
    self.assertEqual(
        _assess(reversed_sample).error,
        pressure_depth_policy.PressureDepthError.TIME_REVERSAL,
    )

  def test_dropout_omits_the_sample(self):
    config = fake_pressure_depth.FakePressureDepthConfig(
        dropout=True, out_of_range=True
    )
    self.assertIsNone(fake_pressure_depth.FakePressureDepth(config).measure())
    absent = pressure_depth_policy.assess_pressure_depth(
        pressure_depth_policy.PressureDepthMeasurementView()
    )
    self.assertEqual(
        absent.error, pressure_depth_policy.PressureDepthError.NONE
    )
    self.assertEqual(
        absent.state, measurement_health_policy.MeasurementStateKind.ABSENT
    )
    self.assertFalse(absent.accepted)

  def test_out_of_range_is_negative_depth_invalid(self):
    config = fake_pressure_depth.FakePressureDepthConfig(
        out_of_range=True, pressure_bias_pa=25.0, depth_bias_m=4.0
    )
    sample = fake_pressure_depth.FakePressureDepth(config).measure()
    self.assertEqual(
        sample.depth_m, fake_pressure_depth.CANONICAL_OUT_OF_RANGE_DEPTH_M
    )
    self.assertEqual(sample.pressure_pa, 200025.0)
    self.assertEqual(
        sample.depth_provenance,
        pressure_depth_pb2.PressureDepthMeasurement.DEPTH_PROVENANCE_FROM_PRESSURE,
    )
    self.assertEqual(sample.fluid_density_kg_m3, 1025.0)
    self.assertEqual(
        sample.health.state, measurement_health_pb2.MeasurementHealth.INVALID
    )
    assessment = _assess(sample)
    self.assertEqual(
        assessment.error, pressure_depth_policy.PressureDepthError.DEPTH
    )
    self.assertEqual(
        assessment.state, measurement_health_policy.MeasurementStateKind.INVALID
    )
    self.assertFalse(assessment.accepted)
    self.assertEqual(
        sample.health.state, measurement_health_pb2.MeasurementHealth.INVALID
    )

  def test_pressure_only_and_direct_depth(self):
    pressure_only = fake_pressure_depth.PressureDepthTruth(
        depth_present=False, provenance_present=False, density_present=False
    )
    pressure = fake_pressure_depth.FakePressureDepth().measure(pressure_only)
    self.assertTrue(pressure.HasField("pressure_pa"))
    self.assertFalse(pressure.HasField("depth_m"))
    self.assertFalse(pressure.HasField("depth_provenance"))
    self.assertFalse(pressure.HasField("fluid_density_kg_m3"))
    self.assertTrue(_assess(pressure).accepted)
    direct = fake_pressure_depth.PressureDepthTruth(
        pressure_present=False, depth_provenance=1, density_present=False
    )
    depth = fake_pressure_depth.FakePressureDepth().measure(direct)
    self.assertFalse(depth.HasField("pressure_pa"))
    self.assertEqual(depth.depth_m, 10.0)
    self.assertEqual(
        depth.depth_provenance,
        pressure_depth_pb2.PressureDepthMeasurement.DEPTH_PROVENANCE_DIRECT,
    )
    self.assertTrue(_assess(depth).accepted)


if __name__ == "__main__":
  unittest.main()
