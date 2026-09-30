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

"""Determinism and fault-injection tests for FakeDvl."""

import unittest

from intrinsic.hardware.marine import dvl_pb2
from intrinsic.hardware.marine import dvl_policy
from intrinsic.hardware.marine import fake_dvl
from intrinsic.hardware.marine import measurement_health_pb2
from intrinsic.hardware.marine import measurement_health_policy


def _view_of(sample):
  health = sample.health
  header = health.header
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
  return dvl_policy.DvlMeasurementView(
      health=health_view,
      mode_present=sample.HasField("mode"),
      mode=sample.mode,
      velocity_x_present=sample.HasField("velocity_x_m_s"),
      velocity_x_m_s=sample.velocity_x_m_s,
      velocity_y_present=sample.HasField("velocity_y_m_s"),
      velocity_y_m_s=sample.velocity_y_m_s,
      velocity_z_present=sample.HasField("velocity_z_m_s"),
      velocity_z_m_s=sample.velocity_z_m_s,
      bottom_lock_present=sample.HasField("bottom_lock"),
      bottom_lock=sample.bottom_lock,
      altitude_present=sample.HasField("altitude_m"),
      altitude_m=sample.altitude_m,
  )


class FakeDvlTest(unittest.TestCase):

  def test_fixed_seed_trace_is_repeatable(self):
    left = fake_dvl.FakeDvl().measure()
    right = fake_dvl.FakeDvl(fake_dvl.FakeDvlConfig()).measure()
    self.assertEqual(left.SerializeToString(), right.SerializeToString())
    self.assertEqual(
        fake_dvl.FakeDvl().measure().SerializeToString(),
        left.SerializeToString(),
    )
    self.assertTrue(dvl_policy.assess_dvl(_view_of(left)).accepted)

  def test_seed_is_the_header_sequence(self):
    sample = fake_dvl.FakeDvl(fake_dvl.FakeDvlConfig(seed=7)).measure()
    self.assertEqual(sample.health.header.sequence, 7)
    other = fake_dvl.FakeDvl(fake_dvl.FakeDvlConfig(seed=8)).measure()
    self.assertNotEqual(sample.SerializeToString(), other.SerializeToString())

  def test_bias_is_degraded_and_added_to_velocity(self):
    sample = fake_dvl.FakeDvl(
        fake_dvl.FakeDvlConfig(bias_x_m_s=0.5, bias_z_m_s=-0.25)
    ).measure(
        fake_dvl.DvlTruth(
            velocity_x_m_s=1.0, velocity_y_m_s=0.25, velocity_z_m_s=0.25
        )
    )
    self.assertEqual(sample.velocity_x_m_s, 1.5)
    self.assertEqual(sample.velocity_y_m_s, 0.25)
    self.assertEqual(sample.velocity_z_m_s, 0.0)
    self.assertEqual(
        sample.health.state, measurement_health_pb2.MeasurementHealth.DEGRADED
    )
    assessment = dvl_policy.assess_dvl(_view_of(sample))
    self.assertEqual(assessment.error, dvl_policy.DvlError.NONE)
    self.assertEqual(
        assessment.state,
        measurement_health_policy.MeasurementStateKind.DEGRADED,
    )
    self.assertFalse(assessment.accepted)

  def test_delay_sets_receive_time_and_reversal_is_not_repaired(self):
    self.assertEqual(
        fake_dvl.add_clock_delay(10, 800000000, 0, 300000000),
        (11, 100000000),
    )
    self.assertEqual(fake_dvl.add_clock_delay(10, 100, 0, -200), (9, 999999900))
    delayed = fake_dvl.FakeDvl(
        fake_dvl.FakeDvlConfig(delay_seconds=2, delay_nanos=0)
    ).measure(fake_dvl.DvlTruth(source_seconds=100, source_nanos=0))
    self.assertEqual(delayed.health.header.receive_time.seconds, 102)
    self.assertTrue(dvl_policy.assess_dvl(_view_of(delayed)).accepted)
    reversed_sample = fake_dvl.FakeDvl(
        fake_dvl.FakeDvlConfig(delay_seconds=-1, delay_nanos=0)
    ).measure(fake_dvl.DvlTruth(source_seconds=100, source_nanos=0))
    self.assertEqual(reversed_sample.health.header.receive_time.seconds, 99)
    self.assertEqual(
        dvl_policy.assess_dvl(_view_of(reversed_sample)).error,
        dvl_policy.DvlError.TIME_REVERSAL,
    )

  def test_dropout_omits_the_sample(self):
    sample = fake_dvl.FakeDvl(
        fake_dvl.FakeDvlConfig(dropout=True, lock_loss=True)
    ).measure()
    self.assertIsNone(sample)
    absent = dvl_policy.assess_dvl(dvl_policy.DvlMeasurementView())
    self.assertEqual(absent.error, dvl_policy.DvlError.NONE)
    self.assertEqual(
        absent.state, measurement_health_policy.MeasurementStateKind.ABSENT
    )
    self.assertFalse(absent.accepted)

  def test_lock_loss_is_bottom_track_invalid(self):
    sample = fake_dvl.FakeDvl(
        fake_dvl.FakeDvlConfig(lock_loss=True, bias_x_m_s=0.25)
    ).measure(fake_dvl.DvlTruth(mode=2, velocity_x_m_s=1.0))
    self.assertEqual(sample.mode, dvl_pb2.DvlMeasurement.MODE_BOTTOM_TRACK)
    self.assertTrue(sample.HasField("bottom_lock"))
    self.assertFalse(sample.bottom_lock)
    self.assertEqual(
        sample.health.state, measurement_health_pb2.MeasurementHealth.INVALID
    )
    self.assertEqual(sample.velocity_x_m_s, 1.25)
    assessment = dvl_policy.assess_dvl(_view_of(sample))
    self.assertEqual(assessment.error, dvl_policy.DvlError.NONE)
    self.assertFalse(assessment.accepted)

  def test_water_track_leaves_bottom_lock_unset(self):
    sample = fake_dvl.FakeDvl().measure(fake_dvl.DvlTruth(mode=2))
    self.assertEqual(sample.mode, dvl_pb2.DvlMeasurement.MODE_WATER_TRACK)
    self.assertFalse(sample.HasField("bottom_lock"))
    self.assertTrue(sample.HasField("altitude_m"))
    self.assertTrue(dvl_policy.assess_dvl(_view_of(sample)).accepted)


if __name__ == "__main__":
  unittest.main()
