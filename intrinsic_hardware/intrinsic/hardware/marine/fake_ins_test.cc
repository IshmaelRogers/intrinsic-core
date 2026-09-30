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

#include "intrinsic/hardware/marine/fake_ins.h"

#include <vector>

#include "gtest/gtest.h"
#include "intrinsic/embodiment/stamped_header_policy.h"
#include "intrinsic/hardware/marine/ins.pb.h"
#include "intrinsic/hardware/marine/ins_policy.h"
#include "intrinsic/hardware/marine/measurement_health.pb.h"

namespace intrinsic::hardware::marine {
namespace {

using intrinsic_proto::hardware::marine::InsSolution;
using intrinsic_proto::hardware::marine::MeasurementHealth;

InsSolutionView ViewOf(const InsSolution& sample,
                       std::vector<double>* covariance,
                       std::vector<SourceHealthView>* sources) {
  InsSolutionView view;
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
  view.position_x_present = sample.has_position_x_m();
  view.position_x_m = sample.position_x_m();
  view.position_y_present = sample.has_position_y_m();
  view.position_y_m = sample.position_y_m();
  view.position_z_present = sample.has_position_z_m();
  view.position_z_m = sample.position_z_m();
  view.orientation_present = sample.has_orientation_xyzw();
  view.orientation_x = sample.orientation_xyzw().x();
  view.orientation_y = sample.orientation_xyzw().y();
  view.orientation_z = sample.orientation_xyzw().z();
  view.orientation_w = sample.orientation_xyzw().w();
  view.linear_velocity_x_present = sample.has_linear_velocity_x_m_s();
  view.linear_velocity_x_m_s = sample.linear_velocity_x_m_s();
  view.linear_velocity_y_present = sample.has_linear_velocity_y_m_s();
  view.linear_velocity_y_m_s = sample.linear_velocity_y_m_s();
  view.linear_velocity_z_present = sample.has_linear_velocity_z_m_s();
  view.linear_velocity_z_m_s = sample.linear_velocity_z_m_s();
  view.angular_velocity_x_present = sample.has_angular_velocity_x_rad_s();
  view.angular_velocity_x_rad_s = sample.angular_velocity_x_rad_s();
  view.angular_velocity_y_present = sample.has_angular_velocity_y_rad_s();
  view.angular_velocity_y_rad_s = sample.angular_velocity_y_rad_s();
  view.angular_velocity_z_present = sample.has_angular_velocity_z_rad_s();
  view.angular_velocity_z_rad_s = sample.angular_velocity_z_rad_s();
  view.source_present = sample.has_source();
  view.source = sample.source();
  return view;
}

InsAssessment Assess(const InsSolution& sample) {
  std::vector<double> covariance;
  std::vector<SourceHealthView> sources;
  return AssessIns(ViewOf(sample, &covariance, &sources));
}

TEST(FakeInsTest, FixedSeedTraceIsRepeatable) {
  FakeIns first;
  FakeIns second(FakeInsConfig{});
  const auto left = first.Measure();
  const auto right = second.Measure();
  ASSERT_TRUE(left.has_value());
  ASSERT_TRUE(right.has_value());
  EXPECT_EQ(left->SerializeAsString(), right->SerializeAsString());
  EXPECT_EQ(first.Measure()->SerializeAsString(), left->SerializeAsString());
  EXPECT_TRUE(Assess(*left).accepted);
  EXPECT_EQ(left->source(), InsSolution::SOURCE_VENDOR_INS);
  EXPECT_EQ(InsSolution::descriptor()->FindFieldByName("magnetic_field_x"),
            nullptr);
  EXPECT_DOUBLE_EQ(InsSignedUnitNoise(42), 0.4831297575436466);
}

TEST(FakeInsTest, BiasDriftAndNoiseAreDegraded) {
  FakeInsConfig bias;
  bias.position_bias_z_m = -0.5;
  bias.linear_velocity_bias_x_m_s = 0.5;
  const auto biased = FakeIns(bias).Measure();
  ASSERT_TRUE(biased.has_value());
  EXPECT_DOUBLE_EQ(biased->position_z_m(), 0.0);
  EXPECT_DOUBLE_EQ(biased->linear_velocity_x_m_s(), 2.0);
  EXPECT_EQ(biased->health().state(), MeasurementHealth::DEGRADED);
  EXPECT_EQ(biased->health().header().validity().state(), 1);
  const InsAssessment assessment = Assess(*biased);
  EXPECT_EQ(assessment.error, InsError::kNone);
  EXPECT_EQ(assessment.state, MeasurementStateKind::kDegraded);
  EXPECT_FALSE(assessment.accepted);
  EXPECT_EQ(biased->health().state(), MeasurementHealth::DEGRADED);

  FakeInsConfig drift;
  drift.position_drift_m_per_seed = 0.25;
  const auto drifted = FakeIns(drift).Measure();
  ASSERT_TRUE(drifted.has_value());
  EXPECT_DOUBLE_EQ(drifted->position_x_m(), 12.0 + 0.25 * 42.0);
  EXPECT_DOUBLE_EQ(drifted->position_y_m(), -4.0 + 0.25 * 42.0);
  EXPECT_EQ(drifted->health().state(), MeasurementHealth::DEGRADED);

  FakeInsConfig noise;
  noise.position_noise_amplitude_m = 0.1;
  const auto noisy = FakeIns(noise).Measure();
  ASSERT_TRUE(noisy.has_value());
  EXPECT_DOUBLE_EQ(noisy->position_x_m(), 12.0 + 0.1 * InsSignedUnitNoise(42));
  EXPECT_EQ(noisy->health().state(), MeasurementHealth::DEGRADED);
}

TEST(FakeInsTest, AbsentTwistDoesNotReceiveRateFaults) {
  FakeInsConfig config;
  config.linear_velocity_bias_x_m_s = 4.0;
  config.angular_velocity_noise_amplitude_rad_s = 1.0;
  InsTruth truth;
  truth.linear_velocity_present = false;
  truth.angular_velocity_present = false;
  const auto sample = FakeIns(config).Measure(truth);
  ASSERT_TRUE(sample.has_value());
  EXPECT_FALSE(sample->has_linear_velocity_x_m_s());
  EXPECT_FALSE(sample->has_angular_velocity_y_rad_s());
  EXPECT_EQ(sample->health().state(), MeasurementHealth::VALID);
  EXPECT_DOUBLE_EQ(sample->position_x_m(), 12.0);
  EXPECT_TRUE(Assess(*sample).accepted);
}

TEST(FakeInsTest, DelayReversalAndDropout) {
  FakeInsConfig config;
  config.delay_seconds = -1;
  config.delay_nanos = 0;
  InsTruth truth;
  truth.source_seconds = 100;
  truth.source_nanos = 0;
  const auto reversed = FakeIns(config).Measure(truth);
  ASSERT_TRUE(reversed.has_value());
  EXPECT_EQ(reversed->health().header().receive_time().seconds(), 99);
  EXPECT_EQ(Assess(*reversed).error, InsError::kTimeReversal);
  EXPECT_EQ(reversed->health().state(), MeasurementHealth::VALID);

  config.dropout = true;
  config.invalid_orientation = true;
  EXPECT_FALSE(FakeIns(config).Measure(truth).has_value());
}

TEST(FakeInsTest, InvalidOrientationIsCanonicalAndWins) {
  FakeInsConfig config;
  config.invalid_orientation = true;
  config.position_bias_x_m = 9.0;
  config.position_drift_m_per_seed = 1.0;
  config.position_noise_amplitude_m = 3.0;
  const auto sample = FakeIns(config).Measure();
  ASSERT_TRUE(sample.has_value());
  EXPECT_DOUBLE_EQ(sample->position_x_m(), 12.0);
  EXPECT_DOUBLE_EQ(sample->orientation_xyzw().w(),
                   kInsCanonicalNonUnitQuaternionW);
  EXPECT_DOUBLE_EQ(sample->orientation_xyzw().x(), 0.0);
  EXPECT_EQ(sample->health().state(), MeasurementHealth::INVALID);
  const InsAssessment assessment = Assess(*sample);
  EXPECT_EQ(assessment.error, InsError::kOrientation);
  EXPECT_EQ(assessment.state, MeasurementStateKind::kInvalid);
  EXPECT_FALSE(assessment.accepted);
  EXPECT_EQ(sample->health().state(), MeasurementHealth::INVALID);
  EXPECT_EQ(sample->health().header().validity().state(), 1);
}

TEST(FakeInsTest, UnspecifiedSourceIsEmittedAndRejected) {
  InsTruth truth;
  truth.source = 0;
  const auto sample = FakeIns().Measure(truth);
  ASSERT_TRUE(sample.has_value());
  EXPECT_TRUE(sample->has_source());
  EXPECT_EQ(sample->source(), InsSolution::SOURCE_UNSPECIFIED);
  EXPECT_EQ(sample->health().state(), MeasurementHealth::VALID);
  const InsAssessment assessment = Assess(*sample);
  EXPECT_EQ(assessment.error, InsError::kSource);
  EXPECT_EQ(assessment.source, InsSourceKind::kUnspecified);
  EXPECT_EQ(sample->source(), InsSolution::SOURCE_UNSPECIFIED);
}

}  // namespace
}  // namespace intrinsic::hardware::marine
