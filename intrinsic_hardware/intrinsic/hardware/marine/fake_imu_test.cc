// Copyright 2026 Intrinsic Innovation LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "intrinsic/hardware/marine/fake_imu.h"

#include <cmath>
#include <vector>

#include "gtest/gtest.h"
#include "intrinsic/embodiment/stamped_header_policy.h"
#include "intrinsic/hardware/marine/imu.pb.h"
#include "intrinsic/hardware/marine/imu_policy.h"
#include "intrinsic/hardware/marine/measurement_health.pb.h"

namespace intrinsic::hardware::marine {
namespace {

using intrinsic_proto::hardware::marine::ImuMeasurement;
using intrinsic_proto::hardware::marine::MeasurementHealth;

ImuMeasurementView ViewOf(const ImuMeasurement& sample,
                          std::vector<double>* covariance,
                          std::vector<SourceHealthView>* sources) {
  ImuMeasurementView view;
  const auto& health = sample.health();
  view.health.header_present = health.has_header();
  view.health.frame_id = health.header().frame_id();
  view.health.source_time_present = health.header().has_source_time();
  view.health.source_time =
      embodiment::ClockReading{health.header().source_time().seconds(),
                               health.header().source_time().nanos()};
  view.health.receive_time_present = health.header().has_receive_time();
  view.health.receive_time =
      embodiment::ClockReading{health.header().receive_time().seconds(),
                               health.header().receive_time().nanos()};
  view.health.header_validity_present = health.header().has_validity();
  view.health.header_validity_state = health.header().validity().state();
  view.health.state_present = health.has_state();
  view.health.state = health.state();
  view.health.quality_present = health.has_quality();
  view.health.quality = health.quality();
  view.health.covariance_present = health.has_covariance();
  if (view.health.covariance_present) {
    covariance->assign(health.covariance().values().begin(),
                       health.covariance().values().end());
    view.health.covariance = *covariance;
  }
  sources->clear();
  for (const auto& source : health.sources()) {
    sources->push_back(SourceHealthView{
        source.source_id(), source.has_validity(), source.validity().state()});
  }
  view.health.sources = *sources;
  view.angular_velocity_x_present = sample.has_angular_velocity_x_rad_s();
  view.angular_velocity_x_rad_s = sample.angular_velocity_x_rad_s();
  view.angular_velocity_y_present = sample.has_angular_velocity_y_rad_s();
  view.angular_velocity_y_rad_s = sample.angular_velocity_y_rad_s();
  view.angular_velocity_z_present = sample.has_angular_velocity_z_rad_s();
  view.angular_velocity_z_rad_s = sample.angular_velocity_z_rad_s();
  view.linear_acceleration_x_present = sample.has_linear_acceleration_x_m_s2();
  view.linear_acceleration_x_m_s2 = sample.linear_acceleration_x_m_s2();
  view.linear_acceleration_y_present = sample.has_linear_acceleration_y_m_s2();
  view.linear_acceleration_y_m_s2 = sample.linear_acceleration_y_m_s2();
  view.linear_acceleration_z_present = sample.has_linear_acceleration_z_m_s2();
  view.linear_acceleration_z_m_s2 = sample.linear_acceleration_z_m_s2();
  view.orientation_present = sample.has_orientation_xyzw();
  view.orientation_x = sample.orientation_xyzw().x();
  view.orientation_y = sample.orientation_xyzw().y();
  view.orientation_z = sample.orientation_xyzw().z();
  view.orientation_w = sample.orientation_xyzw().w();
  return view;
}

ImuAssessment Assess(const ImuMeasurement& sample) {
  std::vector<double> covariance;
  std::vector<SourceHealthView> sources;
  return AssessImu(ViewOf(sample, &covariance, &sources));
}

TEST(FakeImuTest, FixedSeedTraceIsRepeatable) {
  FakeImu first;
  FakeImu second(FakeImuConfig{});
  const auto left = first.Measure();
  const auto right = second.Measure();
  ASSERT_TRUE(left.has_value());
  ASSERT_TRUE(right.has_value());
  EXPECT_EQ(left->SerializeAsString(), right->SerializeAsString());
  EXPECT_EQ(first.Measure()->SerializeAsString(), left->SerializeAsString());
  EXPECT_TRUE(Assess(*left).accepted);
  EXPECT_EQ(ImuMeasurement::descriptor()->FindFieldByName("magnetic_field_x"),
            nullptr);
  EXPECT_DOUBLE_EQ(ImuSignedUnitNoise(42), 0.4831297575436466);
}

TEST(FakeImuTest, SeedIsTheHeaderSequenceAndTheNoiseMix) {
  FakeImuConfig config;
  config.seed = 7;
  config.angular_noise_amplitude_rad_s = 0.1;
  const auto sample = FakeImu(config).Measure();
  ASSERT_TRUE(sample.has_value());
  EXPECT_EQ(sample->health().header().sequence(), 7u);
  EXPECT_DOUBLE_EQ(sample->angular_velocity_x_rad_s(),
                   0.25 + 0.1 * ImuSignedUnitNoise(7));
  EXPECT_DOUBLE_EQ(sample->linear_acceleration_z_m_s2(), 8.0);
  EXPECT_EQ(sample->health().state(), MeasurementHealth::DEGRADED);
  FakeImuConfig other = config;
  other.seed = 8;
  const auto shifted = FakeImu(other).Measure();
  ASSERT_TRUE(shifted.has_value());
  EXPECT_NE(shifted->SerializeAsString(), sample->SerializeAsString());
}

TEST(FakeImuTest, BiasDriftAndNoiseAreDegradedAndNotRepaired) {
  FakeImuConfig bias;
  bias.angular_velocity_bias_z_rad_s = -0.125;
  bias.linear_acceleration_bias_x_m_s2 = 1.5;
  const auto biased = FakeImu(bias).Measure();
  ASSERT_TRUE(biased.has_value());
  EXPECT_DOUBLE_EQ(biased->angular_velocity_z_rad_s(), 0.0);
  EXPECT_DOUBLE_EQ(biased->linear_acceleration_x_m_s2(), 1.5);
  EXPECT_EQ(biased->health().state(), MeasurementHealth::DEGRADED);
  EXPECT_EQ(biased->health().header().validity().state(), 1);
  const ImuAssessment bias_assessment = Assess(*biased);
  EXPECT_EQ(bias_assessment.error, ImuError::kNone);
  EXPECT_EQ(bias_assessment.state, MeasurementStateKind::kDegraded);
  EXPECT_FALSE(bias_assessment.accepted);
  EXPECT_EQ(biased->health().state(), MeasurementHealth::DEGRADED);

  FakeImuConfig drift;
  drift.seed = 42;
  drift.angular_drift_rad_s_per_seed = 0.01;
  drift.linear_drift_m_s2_per_seed = -0.25;
  const auto drifted = FakeImu(drift).Measure();
  ASSERT_TRUE(drifted.has_value());
  EXPECT_DOUBLE_EQ(drifted->angular_velocity_x_rad_s(), 0.25 + 0.01 * 42.0);
  EXPECT_DOUBLE_EQ(drifted->angular_velocity_y_rad_s(), -0.5 + 0.01 * 42.0);
  EXPECT_DOUBLE_EQ(drifted->linear_acceleration_z_m_s2(), 8.0 - 0.25 * 42.0);
  EXPECT_EQ(drifted->health().state(), MeasurementHealth::DEGRADED);

  FakeImuConfig noise;
  noise.linear_noise_amplitude_m_s2 = 0.1;
  const auto noisy = FakeImu(noise).Measure();
  ASSERT_TRUE(noisy.has_value());
  EXPECT_DOUBLE_EQ(noisy->linear_acceleration_z_m_s2(),
                   8.0 + 0.1 * ImuSignedUnitNoise(43));
  EXPECT_DOUBLE_EQ(noisy->angular_velocity_x_rad_s(), 0.25);
  EXPECT_EQ(noisy->health().state(), MeasurementHealth::DEGRADED);
  EXPECT_EQ(Assess(*noisy).state, MeasurementStateKind::kDegraded);
}

TEST(FakeImuTest, DelaySetsReceiveTimeAndReversalIsNotRepaired) {
  FakeImuConfig config;
  config.delay_seconds = 2;
  config.delay_nanos = 0;
  ImuTruth truth;
  truth.source_seconds = 100;
  truth.source_nanos = 0;
  const auto delayed = FakeImu(config).Measure(truth);
  ASSERT_TRUE(delayed.has_value());
  EXPECT_EQ(delayed->health().header().receive_time().seconds(), 102);
  EXPECT_EQ(delayed->health().header().receive_time().nanos(), 0);
  EXPECT_EQ(delayed->health().state(), MeasurementHealth::VALID);
  EXPECT_TRUE(Assess(*delayed).accepted);

  config.delay_seconds = -1;
  config.delay_nanos = 0;
  const auto reversed = FakeImu(config).Measure(truth);
  ASSERT_TRUE(reversed.has_value());
  EXPECT_EQ(reversed->health().header().source_time().seconds(), 100);
  EXPECT_EQ(reversed->health().header().receive_time().seconds(), 99);
  EXPECT_EQ(Assess(*reversed).error, ImuError::kTimeReversal);
  EXPECT_EQ(reversed->health().state(), MeasurementHealth::VALID);
}

TEST(FakeImuTest, DropoutOmitsTheSample) {
  FakeImuConfig config;
  config.dropout = true;
  config.invalid_orientation = true;
  config.angular_noise_amplitude_rad_s = 1.0;
  EXPECT_FALSE(FakeImu(config).Measure().has_value());
  const ImuAssessment absent = AssessImu(ImuMeasurementView{});
  EXPECT_EQ(absent.error, ImuError::kNone);
  EXPECT_EQ(absent.state, MeasurementStateKind::kAbsent);
  EXPECT_FALSE(absent.accepted);
}

TEST(FakeImuTest, RatesOnlyOmitsOrientation) {
  ImuTruth truth;
  truth.orientation_present = false;
  const auto sample = FakeImu().Measure(truth);
  ASSERT_TRUE(sample.has_value());
  EXPECT_FALSE(sample->has_orientation_xyzw());
  EXPECT_TRUE(Assess(*sample).accepted);
}

TEST(FakeImuTest, InvalidOrientationIsCanonicalAndWins) {
  FakeImuConfig config;
  config.invalid_orientation = true;
  config.angular_velocity_bias_x_rad_s = 3.0;
  config.angular_drift_rad_s_per_seed = 1.0;
  config.linear_noise_amplitude_m_s2 = 4.0;
  const auto sample = FakeImu(config).Measure();
  ASSERT_TRUE(sample.has_value());
  EXPECT_DOUBLE_EQ(sample->angular_velocity_x_rad_s(), 0.25);
  EXPECT_DOUBLE_EQ(sample->linear_acceleration_z_m_s2(), 8.0);
  EXPECT_DOUBLE_EQ(sample->orientation_xyzw().x(),
                   kImuCanonicalNonUnitQuaternionX);
  EXPECT_DOUBLE_EQ(sample->orientation_xyzw().y(),
                   kImuCanonicalNonUnitQuaternionY);
  EXPECT_DOUBLE_EQ(sample->orientation_xyzw().z(),
                   kImuCanonicalNonUnitQuaternionZ);
  EXPECT_DOUBLE_EQ(sample->orientation_xyzw().w(),
                   kImuCanonicalNonUnitQuaternionW);
  EXPECT_EQ(sample->health().state(), MeasurementHealth::INVALID);
  EXPECT_EQ(sample->health().header().validity().state(), 1);
  const ImuAssessment assessment = Assess(*sample);
  EXPECT_EQ(assessment.error, ImuError::kOrientation);
  EXPECT_EQ(assessment.state, MeasurementStateKind::kInvalid);
  EXPECT_FALSE(assessment.accepted);
  EXPECT_EQ(sample->health().state(), MeasurementHealth::INVALID);
}

}  // namespace
}  // namespace intrinsic::hardware::marine
