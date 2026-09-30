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

"""Determinism and fault-injection tests for FakeImu."""

import unittest

from intrinsic.hardware.marine import fake_imu
from intrinsic.hardware.marine import imu_pb2
from intrinsic.hardware.marine import imu_policy
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
  return imu_policy.ImuMeasurementView(
      health=health_view,
      angular_velocity_x_present=sample.HasField("angular_velocity_x_rad_s"),
      angular_velocity_x_rad_s=sample.angular_velocity_x_rad_s,
      angular_velocity_y_present=sample.HasField("angular_velocity_y_rad_s"),
      angular_velocity_y_rad_s=sample.angular_velocity_y_rad_s,
      angular_velocity_z_present=sample.HasField("angular_velocity_z_rad_s"),
      angular_velocity_z_rad_s=sample.angular_velocity_z_rad_s,
      linear_acceleration_x_present=sample.HasField(
          "linear_acceleration_x_m_s2"
      ),
      linear_acceleration_x_m_s2=sample.linear_acceleration_x_m_s2,
      linear_acceleration_y_present=sample.HasField(
          "linear_acceleration_y_m_s2"
      ),
      linear_acceleration_y_m_s2=sample.linear_acceleration_y_m_s2,
      linear_acceleration_z_present=sample.HasField(
          "linear_acceleration_z_m_s2"
      ),
      linear_acceleration_z_m_s2=sample.linear_acceleration_z_m_s2,
      orientation_present=sample.HasField("orientation_xyzw"),
      orientation_x=orientation.x,
      orientation_y=orientation.y,
      orientation_z=orientation.z,
      orientation_w=orientation.w,
  )


def _assess(sample):
  return imu_policy.assess_imu(_view_of(sample))


class FakeImuTest(unittest.TestCase):

  def test_fixed_seed_trace_is_repeatable(self):
    left = fake_imu.FakeImu().measure()
    right = fake_imu.FakeImu(fake_imu.FakeImuConfig()).measure()
    self.assertEqual(left.SerializeToString(), right.SerializeToString())
    self.assertTrue(_assess(left).accepted)
    self.assertIsNone(
        imu_pb2.ImuMeasurement.DESCRIPTOR.fields_by_name.get("magnetic_field_x")
    )
    self.assertEqual(fake_imu.imu_signed_unit_noise(42), 0.4831297575436466)

  def test_seed_is_the_header_sequence_and_the_noise_mix(self):
    config = fake_imu.FakeImuConfig(seed=7, angular_noise_amplitude_rad_s=0.1)
    sample = fake_imu.FakeImu(config).measure()
    self.assertEqual(sample.health.header.sequence, 7)
    self.assertEqual(
        sample.angular_velocity_x_rad_s,
        0.25 + 0.1 * fake_imu.imu_signed_unit_noise(7),
    )
    self.assertEqual(sample.linear_acceleration_z_m_s2, 8.0)
    self.assertEqual(
        sample.health.state, measurement_health_pb2.MeasurementHealth.DEGRADED
    )

  def test_bias_drift_and_noise_are_degraded(self):
    biased = fake_imu.FakeImu(
        fake_imu.FakeImuConfig(
            angular_velocity_bias_z_rad_s=-0.125,
            linear_acceleration_bias_x_m_s2=1.5,
        )
    ).measure()
    self.assertEqual(biased.angular_velocity_z_rad_s, 0.0)
    self.assertEqual(biased.linear_acceleration_x_m_s2, 1.5)
    self.assertEqual(
        biased.health.state, measurement_health_pb2.MeasurementHealth.DEGRADED
    )
    assessment = _assess(biased)
    self.assertEqual(assessment.error, imu_policy.ImuError.NONE)
    self.assertEqual(
        assessment.state,
        measurement_health_policy.MeasurementStateKind.DEGRADED,
    )
    self.assertFalse(assessment.accepted)
    self.assertEqual(biased.health.header.validity.state, 1)
    drifted = fake_imu.FakeImu(
        fake_imu.FakeImuConfig(
            angular_drift_rad_s_per_seed=0.01,
            linear_drift_m_s2_per_seed=-0.25,
        )
    ).measure()
    self.assertEqual(drifted.angular_velocity_x_rad_s, 0.25 + 0.01 * 42.0)
    self.assertEqual(drifted.linear_acceleration_z_m_s2, 8.0 - 0.25 * 42.0)
    noisy = fake_imu.FakeImu(
        fake_imu.FakeImuConfig(linear_noise_amplitude_m_s2=0.1)
    ).measure()
    self.assertEqual(
        noisy.linear_acceleration_z_m_s2,
        8.0 + 0.1 * fake_imu.imu_signed_unit_noise(43),
    )
    self.assertEqual(noisy.angular_velocity_x_rad_s, 0.25)

  def test_delay_reversal_dropout_and_rates_only(self):
    truth = fake_imu.ImuTruth(source_seconds=100, source_nanos=0)
    delayed = fake_imu.FakeImu(
        fake_imu.FakeImuConfig(delay_seconds=2, delay_nanos=0)
    ).measure(truth)
    self.assertEqual(delayed.health.header.receive_time.seconds, 102)
    self.assertTrue(_assess(delayed).accepted)
    reversed_sample = fake_imu.FakeImu(
        fake_imu.FakeImuConfig(delay_seconds=-1, delay_nanos=0)
    ).measure(truth)
    self.assertEqual(reversed_sample.health.header.receive_time.seconds, 99)
    self.assertEqual(
        _assess(reversed_sample).error, imu_policy.ImuError.TIME_REVERSAL
    )
    self.assertEqual(
        reversed_sample.health.state,
        measurement_health_pb2.MeasurementHealth.VALID,
    )
    self.assertIsNone(
        fake_imu.FakeImu(
            fake_imu.FakeImuConfig(dropout=True, invalid_orientation=True)
        ).measure()
    )
    rates = fake_imu.FakeImu().measure(
        fake_imu.ImuTruth(orientation_present=False)
    )
    self.assertFalse(rates.HasField("orientation_xyzw"))
    self.assertTrue(_assess(rates).accepted)

  def test_invalid_orientation_is_canonical_and_wins(self):
    sample = fake_imu.FakeImu(
        fake_imu.FakeImuConfig(
            invalid_orientation=True,
            angular_velocity_bias_x_rad_s=3.0,
            angular_drift_rad_s_per_seed=1.0,
            linear_noise_amplitude_m_s2=4.0,
        )
    ).measure()
    self.assertEqual(sample.angular_velocity_x_rad_s, 0.25)
    self.assertEqual(sample.orientation_xyzw.w, 2.0)
    self.assertEqual(
        sample.health.state, measurement_health_pb2.MeasurementHealth.INVALID
    )
    assessment = _assess(sample)
    self.assertEqual(assessment.error, imu_policy.ImuError.ORIENTATION)
    self.assertEqual(
        assessment.state, measurement_health_policy.MeasurementStateKind.INVALID
    )
    self.assertEqual(
        sample.health.state, measurement_health_pb2.MeasurementHealth.INVALID
    )
    self.assertEqual(sample.health.header.validity.state, 1)


if __name__ == "__main__":
  unittest.main()
