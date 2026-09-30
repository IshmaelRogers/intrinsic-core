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

#include "intrinsic/hardware/marine/fake_dvl.h"

#include <vector>

#include "gtest/gtest.h"
#include "intrinsic/embodiment/stamped_header_policy.h"
#include "intrinsic/hardware/marine/dvl.pb.h"
#include "intrinsic/hardware/marine/dvl_policy.h"
#include "intrinsic/hardware/marine/measurement_health.pb.h"

namespace intrinsic::hardware::marine {
namespace {

using intrinsic_proto::hardware::marine::DvlMeasurement;
using intrinsic_proto::hardware::marine::MeasurementHealth;

DvlMeasurementView ViewOf(const DvlMeasurement& sample,
                          std::vector<double>* covariance,
                          std::vector<SourceHealthView>* sources) {
  DvlMeasurementView view;
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
  view.mode_present = sample.has_mode();
  view.mode = sample.mode();
  view.velocity_x_present = sample.has_velocity_x_m_s();
  view.velocity_x_m_s = sample.velocity_x_m_s();
  view.velocity_y_present = sample.has_velocity_y_m_s();
  view.velocity_y_m_s = sample.velocity_y_m_s();
  view.velocity_z_present = sample.has_velocity_z_m_s();
  view.velocity_z_m_s = sample.velocity_z_m_s();
  view.bottom_lock_present = sample.has_bottom_lock();
  view.bottom_lock = sample.bottom_lock();
  view.altitude_present = sample.has_altitude_m();
  view.altitude_m = sample.altitude_m();
  return view;
}

DvlAssessment Assess(const DvlMeasurement& sample) {
  std::vector<double> covariance;
  std::vector<SourceHealthView> sources;
  return AssessDvl(ViewOf(sample, &covariance, &sources));
}

TEST(FakeDvlTest, FixedSeedTraceIsRepeatable) {
  FakeDvl first;
  FakeDvl second(FakeDvlConfig{});
  const auto left = first.Measure();
  const auto right = second.Measure();
  ASSERT_TRUE(left.has_value());
  ASSERT_TRUE(right.has_value());
  EXPECT_EQ(left->SerializeAsString(), right->SerializeAsString());
  EXPECT_EQ(first.Measure()->SerializeAsString(), left->SerializeAsString());
  EXPECT_TRUE(Assess(*left).accepted);
}

TEST(FakeDvlTest, SeedIsTheHeaderSequence) {
  FakeDvlConfig config;
  config.seed = 7;
  const auto sample = FakeDvl(config).Measure();
  ASSERT_TRUE(sample.has_value());
  EXPECT_EQ(sample->health().header().sequence(), 7u);
  FakeDvlConfig other = config;
  other.seed = 8;
  EXPECT_NE(FakeDvl(other).Measure()->SerializeAsString(),
            sample->SerializeAsString());
}

TEST(FakeDvlTest, BiasIsDegradedAndAddedToVelocity) {
  FakeDvlConfig config;
  config.bias_x_m_s = 0.5;
  config.bias_y_m_s = 0.0;
  config.bias_z_m_s = -0.25;
  DvlTruth truth;
  truth.velocity_x_m_s = 1.0;
  truth.velocity_y_m_s = 0.25;
  truth.velocity_z_m_s = 0.25;
  const auto sample = FakeDvl(config).Measure(truth);
  ASSERT_TRUE(sample.has_value());
  EXPECT_DOUBLE_EQ(sample->velocity_x_m_s(), 1.5);
  EXPECT_DOUBLE_EQ(sample->velocity_y_m_s(), 0.25);
  EXPECT_DOUBLE_EQ(sample->velocity_z_m_s(), 0.0);
  EXPECT_EQ(sample->health().state(), MeasurementHealth::DEGRADED);
  EXPECT_EQ(sample->health().header().validity().state(), 1);
  const DvlAssessment assessment = Assess(*sample);
  EXPECT_EQ(assessment.error, DvlError::kNone);
  EXPECT_EQ(assessment.state, MeasurementStateKind::kDegraded);
  EXPECT_FALSE(assessment.accepted);
  EXPECT_EQ(sample->health().state(), MeasurementHealth::DEGRADED);
}

TEST(FakeDvlTest, DelaySetsReceiveTimeAndReversalIsNotRepaired) {
  int64_t seconds = 10;
  int32_t nanos = 800000000;
  AddClockDelay(&seconds, &nanos, 0, 300000000);
  EXPECT_EQ(seconds, 11);
  EXPECT_EQ(nanos, 100000000);

  seconds = 10;
  nanos = 100;
  AddClockDelay(&seconds, &nanos, 0, -200);
  EXPECT_EQ(seconds, 9);
  EXPECT_EQ(nanos, 999999900);

  FakeDvlConfig config;
  config.delay_seconds = 2;
  config.delay_nanos = 0;
  DvlTruth truth;
  truth.source_seconds = 100;
  truth.source_nanos = 0;
  const auto delayed = FakeDvl(config).Measure(truth);
  ASSERT_TRUE(delayed.has_value());
  EXPECT_EQ(delayed->health().header().receive_time().seconds(), 102);
  EXPECT_EQ(delayed->health().header().receive_time().nanos(), 0);
  EXPECT_EQ(delayed->health().state(), MeasurementHealth::VALID);
  EXPECT_TRUE(Assess(*delayed).accepted);

  config.delay_seconds = -1;
  config.delay_nanos = 0;
  const auto reversed = FakeDvl(config).Measure(truth);
  ASSERT_TRUE(reversed.has_value());
  EXPECT_EQ(reversed->health().header().source_time().seconds(), 100);
  EXPECT_EQ(reversed->health().header().receive_time().seconds(), 99);
  EXPECT_EQ(Assess(*reversed).error, DvlError::kTimeReversal);
}

TEST(FakeDvlTest, DropoutOmitsTheSample) {
  FakeDvlConfig config;
  config.dropout = true;
  config.lock_loss = true;
  EXPECT_FALSE(FakeDvl(config).Measure().has_value());
  const DvlAssessment absent = AssessDvl(DvlMeasurementView{});
  EXPECT_EQ(absent.error, DvlError::kNone);
  EXPECT_EQ(absent.state, MeasurementStateKind::kAbsent);
  EXPECT_FALSE(absent.accepted);
}

TEST(FakeDvlTest, LockLossIsBottomTrackInvalid) {
  FakeDvlConfig config;
  config.lock_loss = true;
  config.bias_x_m_s = 0.25;
  DvlTruth truth;
  truth.mode = 2;
  truth.velocity_x_m_s = 1.0;
  const auto sample = FakeDvl(config).Measure(truth);
  ASSERT_TRUE(sample.has_value());
  EXPECT_EQ(sample->mode(), DvlMeasurement::MODE_BOTTOM_TRACK);
  EXPECT_TRUE(sample->has_bottom_lock());
  EXPECT_FALSE(sample->bottom_lock());
  EXPECT_EQ(sample->health().state(), MeasurementHealth::INVALID);
  EXPECT_DOUBLE_EQ(sample->velocity_x_m_s(), 1.25);
  const DvlAssessment assessment = Assess(*sample);
  EXPECT_EQ(assessment.error, DvlError::kNone);
  EXPECT_EQ(assessment.state, MeasurementStateKind::kInvalid);
  EXPECT_FALSE(assessment.accepted);
}

TEST(FakeDvlTest, WaterTrackLeavesBottomLockUnset) {
  DvlTruth truth;
  truth.mode = 2;
  const auto sample = FakeDvl().Measure(truth);
  ASSERT_TRUE(sample.has_value());
  EXPECT_EQ(sample->mode(), DvlMeasurement::MODE_WATER_TRACK);
  EXPECT_FALSE(sample->has_bottom_lock());
  EXPECT_TRUE(sample->has_altitude_m());
  EXPECT_TRUE(Assess(*sample).accepted);
}

}  // namespace
}  // namespace intrinsic::hardware::marine
