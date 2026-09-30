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

"""Determinism and fault-injection tests for FakeIns."""

import unittest

from intrinsic.hardware.marine import fake_ins
from intrinsic.hardware.marine import ins_pb2
from intrinsic.hardware.marine import ins_policy
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
  orientation = sample.orientation_xyzw
  return ins_policy.InsSolutionView(
      health=health_view,
      position_x_present=sample.HasField("position_x_m"),
      position_x_m=sample.position_x_m,
      position_y_present=sample.HasField("position_y_m"),
      position_y_m=sample.position_y_m,
      position_z_present=sample.HasField("position_z_m"),
      position_z_m=sample.position_z_m,
      orientation_present=sample.HasField("orientation_xyzw"),
      orientation_x=orientation.x,
      orientation_y=orientation.y,
      orientation_z=orientation.z,
      orientation_w=orientation.w,
      linear_velocity_x_present=sample.HasField("linear_velocity_x_m_s"),
      linear_velocity_x_m_s=sample.linear_velocity_x_m_s,
      linear_velocity_y_present=sample.HasField("linear_velocity_y_m_s"),
      linear_velocity_y_m_s=sample.linear_velocity_y_m_s,
      linear_velocity_z_present=sample.HasField("linear_velocity_z_m_s"),
      linear_velocity_z_m_s=sample.linear_velocity_z_m_s,
      angular_velocity_x_present=sample.HasField("angular_velocity_x_rad_s"),
      angular_velocity_x_rad_s=sample.angular_velocity_x_rad_s,
      angular_velocity_y_present=sample.HasField("angular_velocity_y_rad_s"),
      angular_velocity_y_rad_s=sample.angular_velocity_y_rad_s,
      angular_velocity_z_present=sample.HasField("angular_velocity_z_rad_s"),
      angular_velocity_z_rad_s=sample.angular_velocity_z_rad_s,
      source_present=sample.HasField("source"),
      source=sample.source,
  )


def _assess(sample):
  return ins_policy.assess_ins(_view_of(sample))


class FakeInsTest(unittest.TestCase):

  def test_fixed_seed_trace_is_repeatable(self):
    left = fake_ins.FakeIns().measure()
    right = fake_ins.FakeIns(fake_ins.FakeInsConfig()).measure()
    self.assertEqual(left.SerializeToString(), right.SerializeToString())
    self.assertTrue(_assess(left).accepted)
    self.assertEqual(left.source, ins_pb2.InsSolution.SOURCE_VENDOR_INS)
    self.assertIsNone(
        ins_pb2.InsSolution.DESCRIPTOR.fields_by_name.get("magnetic_field_x")
    )
    self.assertEqual(fake_ins.ins_signed_unit_noise(42), 0.4831297575436466)

  def test_bias_drift_and_noise_are_degraded(self):
    biased = fake_ins.FakeIns(
        fake_ins.FakeInsConfig(
            position_bias_z_m=-0.5,
            linear_velocity_bias_x_m_s=0.5,
        )
    ).measure()
    self.assertEqual(biased.position_z_m, 0.0)
    self.assertEqual(biased.linear_velocity_x_m_s, 2.0)
    self.assertEqual(
        biased.health.state, measurement_health_pb2.MeasurementHealth.DEGRADED
    )
    assessment = _assess(biased)
    self.assertEqual(assessment.error, ins_policy.InsError.NONE)
    self.assertFalse(assessment.accepted)
    self.assertEqual(biased.health.header.validity.state, 1)
    drifted = fake_ins.FakeIns(
        fake_ins.FakeInsConfig(position_drift_m_per_seed=0.25)
    ).measure()
    self.assertEqual(drifted.position_x_m, 12.0 + 0.25 * 42.0)
    noisy = fake_ins.FakeIns(
        fake_ins.FakeInsConfig(position_noise_amplitude_m=0.1)
    ).measure()
    self.assertEqual(
        noisy.position_x_m, 12.0 + 0.1 * fake_ins.ins_signed_unit_noise(42)
    )

  def test_absent_twist_does_not_receive_rate_faults(self):
    sample = fake_ins.FakeIns(
        fake_ins.FakeInsConfig(
            linear_velocity_bias_x_m_s=4.0,
            angular_velocity_noise_amplitude_rad_s=1.0,
        )
    ).measure(
        fake_ins.InsTruth(
            linear_velocity_present=False,
            angular_velocity_present=False,
        )
    )
    self.assertFalse(sample.HasField("linear_velocity_x_m_s"))
    self.assertFalse(sample.HasField("angular_velocity_y_rad_s"))
    self.assertEqual(
        sample.health.state, measurement_health_pb2.MeasurementHealth.VALID
    )
    self.assertEqual(sample.position_x_m, 12.0)
    self.assertTrue(_assess(sample).accepted)

  def test_delay_reversal_and_dropout(self):
    truth = fake_ins.InsTruth(source_seconds=100, source_nanos=0)
    reversed_sample = fake_ins.FakeIns(
        fake_ins.FakeInsConfig(delay_seconds=-1, delay_nanos=0)
    ).measure(truth)
    self.assertEqual(reversed_sample.health.header.receive_time.seconds, 99)
    self.assertEqual(
        _assess(reversed_sample).error, ins_policy.InsError.TIME_REVERSAL
    )
    self.assertIsNone(
        fake_ins.FakeIns(
            fake_ins.FakeInsConfig(dropout=True, invalid_orientation=True)
        ).measure(truth)
    )

  def test_invalid_orientation_is_canonical_and_wins(self):
    sample = fake_ins.FakeIns(
        fake_ins.FakeInsConfig(
            invalid_orientation=True,
            position_bias_x_m=9.0,
            position_drift_m_per_seed=1.0,
            position_noise_amplitude_m=3.0,
        )
    ).measure()
    self.assertEqual(sample.position_x_m, 12.0)
    self.assertEqual(sample.orientation_xyzw.w, 2.0)
    self.assertEqual(
        sample.health.state, measurement_health_pb2.MeasurementHealth.INVALID
    )
    assessment = _assess(sample)
    self.assertEqual(assessment.error, ins_policy.InsError.ORIENTATION)
    self.assertEqual(
        assessment.state, measurement_health_policy.MeasurementStateKind.INVALID
    )
    self.assertEqual(
        sample.health.state, measurement_health_pb2.MeasurementHealth.INVALID
    )
    self.assertEqual(sample.health.header.validity.state, 1)

  def test_unspecified_source_is_emitted_and_rejected(self):
    sample = fake_ins.FakeIns().measure(fake_ins.InsTruth(source=0))
    self.assertTrue(sample.HasField("source"))
    self.assertEqual(sample.source, ins_pb2.InsSolution.SOURCE_UNSPECIFIED)
    assessment = _assess(sample)
    self.assertEqual(assessment.error, ins_policy.InsError.SOURCE)
    self.assertEqual(assessment.source, ins_policy.InsSourceKind.UNSPECIFIED)
    self.assertEqual(sample.source, ins_pb2.InsSolution.SOURCE_UNSPECIFIED)


if __name__ == "__main__":
  unittest.main()
