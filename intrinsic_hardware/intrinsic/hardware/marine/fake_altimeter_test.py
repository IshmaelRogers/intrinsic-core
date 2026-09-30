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

"""Determinism and fault-injection tests for FakeAltimeter."""

import dataclasses
import unittest

from intrinsic.hardware.marine import altimeter_pb2
from intrinsic.hardware.marine import altimeter_policy
from intrinsic.hardware.marine import fake_altimeter
from intrinsic.hardware.marine import measurement_health_pb2
from intrinsic.hardware.marine import measurement_health_policy


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
  return altimeter_policy.AltimeterMeasurementView(
      health=health_view,
      range_present=sample.HasField("range_m"),
      range_m=sample.range_m,
      beam_present=sample.HasField("beam_id"),
      beam_id=sample.beam_id,
      min_present=sample.HasField("min_range_m"),
      min_range_m=sample.min_range_m,
      max_present=sample.HasField("max_range_m"),
      max_range_m=sample.max_range_m,
      has_return_present=sample.HasField("has_return"),
      has_return=sample.has_return,
  )


def _assess(sample):
  return altimeter_policy.assess_altimeter(_view_of(sample))


class FakeAltimeterTest(unittest.TestCase):

  def test_fixed_seed_trace_is_repeatable(self):
    first = fake_altimeter.FakeAltimeter()
    second = fake_altimeter.FakeAltimeter(fake_altimeter.FakeAltimeterConfig())
    left = first.measure()
    right = second.measure()
    self.assertEqual(left.SerializeToString(), right.SerializeToString())
    self.assertEqual(
        first.measure().SerializeToString(), left.SerializeToString()
    )
    self.assertTrue(_assess(left).accepted)
    self.assertIsNone(
        altimeter_pb2.AltimeterMeasurement.DESCRIPTOR.fields_by_name.get(
            "altitude_m"
        )
    )

  def test_seed_is_the_header_sequence_and_the_noise_mix(self):
    config = fake_altimeter.FakeAltimeterConfig(seed=7, noise_amplitude_m=0.1)
    sample = fake_altimeter.FakeAltimeter(config).measure()
    self.assertEqual(sample.health.header.sequence, 7)
    self.assertEqual(
        sample.range_m,
        10.0 + 0.1 * fake_altimeter.altimeter_signed_unit_noise(7),
    )
    other = fake_altimeter.FakeAltimeter(dataclasses.replace(config, seed=8))
    shifted = other.measure()
    self.assertNotEqual(shifted.SerializeToString(), sample.SerializeToString())
    self.assertNotEqual(shifted.range_m, sample.range_m)

  def test_noise_is_degraded_and_not_repaired(self):
    self.assertEqual(
        fake_altimeter.altimeter_signed_unit_noise(42), 0.4831297575436466
    )
    config = fake_altimeter.FakeAltimeterConfig(noise_amplitude_m=0.1)
    sample = fake_altimeter.FakeAltimeter(config).measure()
    self.assertEqual(sample.range_m, 10.0 + 0.1 * 0.4831297575436466)
    self.assertEqual(
        sample.health.state, measurement_health_pb2.MeasurementHealth.DEGRADED
    )
    self.assertEqual(sample.health.header.validity.state, 1)
    assessment = _assess(sample)
    self.assertEqual(assessment.error, altimeter_policy.AltimeterError.NONE)
    self.assertEqual(
        assessment.state,
        measurement_health_policy.MeasurementStateKind.DEGRADED,
    )
    self.assertFalse(assessment.accepted)
    negative = fake_altimeter.FakeAltimeter(
        dataclasses.replace(config, noise_amplitude_m=-30.0)
    ).measure()
    self.assertLess(negative.range_m, 0.0)
    self.assertEqual(
        negative.health.state, measurement_health_pb2.MeasurementHealth.DEGRADED
    )
    rejected = _assess(negative)
    self.assertEqual(rejected.error, altimeter_policy.AltimeterError.RANGE)
    self.assertEqual(
        rejected.state, measurement_health_policy.MeasurementStateKind.DEGRADED
    )
    self.assertEqual(
        negative.health.state, measurement_health_pb2.MeasurementHealth.DEGRADED
    )

  def test_delay_sets_receive_time_and_reversal_is_not_repaired(self):
    config = fake_altimeter.FakeAltimeterConfig(delay_seconds=2, delay_nanos=0)
    truth = fake_altimeter.AltimeterTruth(source_seconds=100, source_nanos=0)
    delayed = fake_altimeter.FakeAltimeter(config).measure(truth)
    self.assertEqual(delayed.health.header.receive_time.seconds, 102)
    self.assertEqual(delayed.health.header.receive_time.nanos, 0)
    self.assertEqual(
        delayed.health.state, measurement_health_pb2.MeasurementHealth.VALID
    )
    self.assertTrue(_assess(delayed).accepted)
    reversed_sample = fake_altimeter.FakeAltimeter(
        dataclasses.replace(config, delay_seconds=-1)
    ).measure(truth)
    self.assertEqual(reversed_sample.health.header.source_time.seconds, 100)
    self.assertEqual(reversed_sample.health.header.receive_time.seconds, 99)
    self.assertEqual(
        _assess(reversed_sample).error,
        altimeter_policy.AltimeterError.TIME_REVERSAL,
    )

  def test_dropout_omits_the_sample(self):
    config = fake_altimeter.FakeAltimeterConfig(
        dropout=True, no_return=True, out_of_range=True
    )
    self.assertIsNone(fake_altimeter.FakeAltimeter(config).measure())
    absent = altimeter_policy.assess_altimeter(
        altimeter_policy.AltimeterMeasurementView()
    )
    self.assertEqual(absent.error, altimeter_policy.AltimeterError.NONE)
    self.assertEqual(
        absent.state, measurement_health_policy.MeasurementStateKind.ABSENT
    )
    self.assertFalse(absent.accepted)

  def test_no_return_omits_range_and_is_invalid(self):
    config = fake_altimeter.FakeAltimeterConfig(
        no_return=True, out_of_range=True, noise_amplitude_m=4.0
    )
    sample = fake_altimeter.FakeAltimeter(config).measure()
    self.assertFalse(sample.HasField("range_m"))
    self.assertTrue(sample.HasField("has_return"))
    self.assertFalse(sample.has_return)
    self.assertEqual(sample.beam_id, "down")
    self.assertEqual(sample.min_range_m, 0.5)
    self.assertEqual(sample.max_range_m, 100.0)
    self.assertEqual(
        sample.health.state, measurement_health_pb2.MeasurementHealth.INVALID
    )
    assessment = _assess(sample)
    self.assertEqual(assessment.error, altimeter_policy.AltimeterError.NONE)
    self.assertEqual(
        assessment.state, measurement_health_policy.MeasurementStateKind.INVALID
    )
    self.assertFalse(assessment.accepted)

  def test_out_of_range_is_canonical_fixture_invalid(self):
    config = fake_altimeter.FakeAltimeterConfig(
        out_of_range=True, noise_amplitude_m=4.0
    )
    truth = fake_altimeter.AltimeterTruth(
        range_m=10.0, min_range_m=1.0, max_range_m=20.0
    )
    sample = fake_altimeter.FakeAltimeter(config).measure(truth)
    self.assertEqual(
        sample.range_m, fake_altimeter.CANONICAL_OUT_OF_RANGE_RANGE_M
    )
    self.assertEqual(sample.min_range_m, fake_altimeter.CANONICAL_MIN_RANGE_M)
    self.assertEqual(sample.max_range_m, fake_altimeter.CANONICAL_MAX_RANGE_M)
    self.assertTrue(sample.has_return)
    self.assertEqual(sample.beam_id, "down")
    self.assertEqual(
        sample.health.state, measurement_health_pb2.MeasurementHealth.INVALID
    )
    assessment = _assess(sample)
    self.assertEqual(assessment.error, altimeter_policy.AltimeterError.BOUNDS)
    self.assertEqual(
        assessment.state, measurement_health_policy.MeasurementStateKind.INVALID
    )
    self.assertFalse(assessment.accepted)

  def test_invalid_quality_is_not_rewritten(self):
    sample = fake_altimeter.FakeAltimeter().measure(
        fake_altimeter.AltimeterTruth(quality=1.5)
    )
    self.assertEqual(sample.health.quality, 1.5)
    self.assertEqual(
        sample.health.state, measurement_health_pb2.MeasurementHealth.VALID
    )
    self.assertEqual(
        _assess(sample).error, altimeter_policy.AltimeterError.QUALITY
    )
    self.assertEqual(
        sample.health.state, measurement_health_pb2.MeasurementHealth.VALID
    )

  def test_range_only_truth_is_accepted(self):
    sample = fake_altimeter.FakeAltimeter().measure(
        fake_altimeter.AltimeterTruth(
            has_return_present=False, beam_present=False
        )
    )
    self.assertTrue(sample.HasField("range_m"))
    self.assertFalse(sample.HasField("has_return"))
    self.assertFalse(sample.HasField("beam_id"))
    self.assertEqual(sample.range_m, 10.0)
    self.assertTrue(_assess(sample).accepted)


if __name__ == "__main__":
  unittest.main()
