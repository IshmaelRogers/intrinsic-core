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

#include "intrinsic/hardware/marine/fake_surface_fix.h"

#include <cmath>
#include <limits>
#include <optional>
#include <span>
#include <vector>

#include "gtest/gtest.h"
#include "intrinsic/embodiment/stamped_header_policy.h"
#include "intrinsic/hardware/marine/measurement_health.pb.h"
#include "intrinsic/hardware/marine/surface_fix.pb.h"
#include "intrinsic/hardware/marine/surface_fix_policy.h"

namespace intrinsic::hardware::marine {
namespace {

using intrinsic_proto::hardware::marine::MeasurementHealth;
using intrinsic_proto::hardware::marine::SurfaceFix;

SurfaceFixView ViewOf(const SurfaceFix& sample, std::vector<double>* covariance,
                      std::vector<SourceHealthView>* sources) {
  SurfaceFixView view;
  const MeasurementHealth& health = sample.health();
  view.health.header_present = health.has_header();
  view.health.frame_id = health.header().frame_id();
  view.health.source_time_present = health.header().has_source_time();
  if (view.health.source_time_present) {
    view.health.source_time =
        embodiment::ClockReading{health.header().source_time().seconds(),
                                 health.header().source_time().nanos()};
  }
  view.health.receive_time_present = health.header().has_receive_time();
  if (view.health.receive_time_present) {
    view.health.receive_time =
        embodiment::ClockReading{health.header().receive_time().seconds(),
                                 health.header().receive_time().nanos()};
  }
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
  view.source_present = sample.has_source();
  view.source = sample.source();
  view.satellite_count_present = sample.has_satellite_count();
  view.satellite_count = sample.satellite_count();
  view.beacon_count_present = sample.has_beacon_count();
  view.beacon_count = sample.beacon_count();
  view.horizontal_accuracy_present = sample.has_horizontal_accuracy_m();
  view.horizontal_accuracy_m = sample.horizontal_accuracy_m();
  view.vertical_accuracy_present = sample.has_vertical_accuracy_m();
  view.vertical_accuracy_m = sample.vertical_accuracy_m();
  view.velocity_x_present = sample.has_velocity_x_m_s();
  view.velocity_x_m_s = sample.velocity_x_m_s();
  view.velocity_y_present = sample.has_velocity_y_m_s();
  view.velocity_y_m_s = sample.velocity_y_m_s();
  view.velocity_z_present = sample.has_velocity_z_m_s();
  view.velocity_z_m_s = sample.velocity_z_m_s();
  return view;
}

SurfaceFixAssessment Assess(const SurfaceFix& sample) {
  std::vector<double> covariance;
  std::vector<SourceHealthView> sources;
  return AssessSurfaceFix(ViewOf(sample, &covariance, &sources));
}

TEST(FakeSurfaceFixTest, FixedSeedTraceIsRepeatable) {
  FakeSurfaceFix first;
  FakeSurfaceFix second(FakeSurfaceFixConfig{});
  const auto left = first.Measure();
  const auto right = second.Measure();
  ASSERT_TRUE(left.has_value());
  ASSERT_TRUE(right.has_value());
  EXPECT_EQ(left->SerializeAsString(), right->SerializeAsString());
  EXPECT_EQ(first.Measure()->SerializeAsString(), left->SerializeAsString());
  EXPECT_EQ(left->source(), SurfaceFix::FIX_SOURCE_GNSS);
  EXPECT_EQ(left->health().header().frame_id(), "gnss");
  EXPECT_DOUBLE_EQ(left->position_x_m(), 12.5);
  EXPECT_DOUBLE_EQ(left->position_y_m(), -3.25);
  EXPECT_DOUBLE_EQ(left->position_z_m(), 1.0);
  EXPECT_EQ(left->satellite_count(), 12);
  EXPECT_FLOAT_EQ(left->health().quality(), 0.75f);
  EXPECT_EQ(left->health().state(), MeasurementHealth::VALID);
  EXPECT_TRUE(Assess(*left).accepted);
  EXPECT_EQ(SurfaceFix::descriptor()->FindFieldByName("orientation_xyzw"),
            nullptr);
}

TEST(FakeSurfaceFixTest, SeedIsTheHeaderSequence) {
  FakeSurfaceFixConfig config;
  config.seed = 7;
  const auto sample = FakeSurfaceFix(config).Measure();
  ASSERT_TRUE(sample.has_value());
  EXPECT_EQ(sample->health().header().sequence(), 7u);
  config.seed = 8;
  const auto other = FakeSurfaceFix(config).Measure();
  ASSERT_TRUE(other.has_value());
  EXPECT_NE(other->SerializeAsString(), sample->SerializeAsString());
}

TEST(FakeSurfaceFixTest, AcousticTruthIsAccepted) {
  SurfaceFixTruth truth;
  truth.frame_id = "usbl";
  truth.source = 2;
  truth.satellite_count_present = false;
  truth.beacon_count_present = true;
  truth.beacon_count = 4;
  const auto sample = FakeSurfaceFix().Measure(truth);
  ASSERT_TRUE(sample.has_value());
  EXPECT_EQ(sample->source(), SurfaceFix::FIX_SOURCE_ACOUSTIC);
  EXPECT_FALSE(sample->has_satellite_count());
  EXPECT_EQ(sample->beacon_count(), 4);
  EXPECT_TRUE(Assess(*sample).accepted);
}

TEST(FakeSurfaceFixTest, BiasIsAddedDegradedAndNotRepaired) {
  FakeSurfaceFixConfig config;
  config.position_bias_x_m = 0.5;
  config.position_bias_z_m = -2.0;
  const auto sample = FakeSurfaceFix(config).Measure();
  ASSERT_TRUE(sample.has_value());
  EXPECT_DOUBLE_EQ(sample->position_x_m(), 13.0);
  EXPECT_DOUBLE_EQ(sample->position_y_m(), -3.25);
  EXPECT_DOUBLE_EQ(sample->position_z_m(), -1.0);
  EXPECT_EQ(sample->health().state(), MeasurementHealth::DEGRADED);
  const SurfaceFixAssessment assessment = Assess(*sample);
  EXPECT_EQ(assessment.error, SurfaceFixError::kNone);
  EXPECT_EQ(assessment.state, MeasurementStateKind::kDegraded);
  EXPECT_FALSE(assessment.accepted);
}

TEST(FakeSurfaceFixTest, DelaySetsReceiveTimeAndReversalIsNotRepaired) {
  FakeSurfaceFixConfig config;
  config.delay_seconds = 2;
  config.delay_nanos = 0;
  SurfaceFixTruth truth;
  truth.source_seconds = 100;
  truth.source_nanos = 0;
  const auto delayed = FakeSurfaceFix(config).Measure(truth);
  ASSERT_TRUE(delayed.has_value());
  EXPECT_EQ(delayed->health().header().receive_time().seconds(), 102);
  EXPECT_EQ(delayed->health().header().receive_time().nanos(), 0);
  EXPECT_EQ(delayed->health().state(), MeasurementHealth::VALID);
  EXPECT_TRUE(Assess(*delayed).accepted);

  config.delay_seconds = -1;
  const auto reversed = FakeSurfaceFix(config).Measure(truth);
  ASSERT_TRUE(reversed.has_value());
  EXPECT_EQ(reversed->health().header().source_time().seconds(), 100);
  EXPECT_EQ(reversed->health().header().receive_time().seconds(), 99);
  EXPECT_EQ(Assess(*reversed).error, SurfaceFixError::kTimeReversal);
}

TEST(FakeSurfaceFixTest, DropoutOmitsTheSample) {
  FakeSurfaceFixConfig config;
  config.dropout = true;
  config.invalid_fix = true;
  config.position_bias_x_m = 1.0;
  EXPECT_FALSE(FakeSurfaceFix(config).Measure().has_value());
  const SurfaceFixAssessment absent = AssessSurfaceFix(SurfaceFixView{});
  EXPECT_EQ(absent.error, SurfaceFixError::kNone);
  EXPECT_EQ(absent.state, MeasurementStateKind::kAbsent);
  EXPECT_FALSE(absent.accepted);
}

TEST(FakeSurfaceFixTest, InvalidFixIsCanonicalUnspecifiedSourceAndInvalid) {
  FakeSurfaceFixConfig config;
  config.invalid_fix = true;
  config.position_bias_x_m = 4.0;
  const auto sample = FakeSurfaceFix(config).Measure();
  ASSERT_TRUE(sample.has_value());
  EXPECT_TRUE(sample->has_source());
  EXPECT_EQ(sample->source(), SurfaceFix::FIX_SOURCE_UNSPECIFIED);
  EXPECT_EQ(sample->health().state(), MeasurementHealth::INVALID);
  EXPECT_DOUBLE_EQ(sample->position_x_m(), 12.5);
  const SurfaceFixAssessment assessment = Assess(*sample);
  EXPECT_EQ(assessment.error, SurfaceFixError::kSource);
  EXPECT_EQ(assessment.state, MeasurementStateKind::kInvalid);
  EXPECT_FALSE(assessment.accepted);
}

TEST(FakeSurfaceFixTest, InvalidQualityIsNotRewritten) {
  SurfaceFixTruth truth;
  truth.quality = 1.5;
  const auto sample = FakeSurfaceFix().Measure(truth);
  ASSERT_TRUE(sample.has_value());
  EXPECT_FLOAT_EQ(sample->health().quality(), 1.5f);
  EXPECT_EQ(sample->health().state(), MeasurementHealth::VALID);
  EXPECT_EQ(Assess(*sample).error, SurfaceFixError::kQuality);
  EXPECT_EQ(sample->health().state(), MeasurementHealth::VALID);
}

TEST(FakeSurfaceFixTest, BoundaryTruthsAreAccepted) {
  SurfaceFixTruth truth;
  truth.position_x_m = 0.0;
  truth.position_y_m = 0.0;
  truth.position_z_m = 0.0;
  truth.satellite_count = 0;
  truth.horizontal_accuracy_present = true;
  truth.horizontal_accuracy_m = 0.0;
  truth.quality = 0.0;
  const auto sample = FakeSurfaceFix().Measure(truth);
  ASSERT_TRUE(sample.has_value());
  EXPECT_TRUE(sample->has_position_x_m());
  EXPECT_TRUE(sample->has_satellite_count());
  EXPECT_TRUE(sample->has_horizontal_accuracy_m());
  EXPECT_TRUE(Assess(*sample).accepted);
  truth.quality = 1.0;
  EXPECT_TRUE(Assess(*FakeSurfaceFix().Measure(truth)).accepted);
}

TEST(FakeSurfaceFixTest, AbsentCovarianceAndOffSlotTruths) {
  SurfaceFixTruth truth;
  truth.covariance_present = false;
  const auto absent = FakeSurfaceFix().Measure(truth);
  ASSERT_TRUE(absent.has_value());
  EXPECT_FALSE(absent->health().has_covariance());
  EXPECT_TRUE(Assess(*absent).accepted);

  auto sample = FakeSurfaceFix().Measure();
  ASSERT_TRUE(sample.has_value());
  sample->mutable_health()->mutable_covariance()->set_values(1, 0.5);
  sample->mutable_health()->mutable_covariance()->set_values(6, 0.5);
  EXPECT_EQ(Assess(*sample).error, SurfaceFixError::kCovarianceSlots);
}

TEST(FakeSurfaceFixTest, MissingSourceTruthIsRejectedAndStaysValid) {
  SurfaceFixTruth truth;
  truth.source_present = false;
  const auto sample = FakeSurfaceFix().Measure(truth);
  ASSERT_TRUE(sample.has_value());
  EXPECT_FALSE(sample->has_source());
  EXPECT_EQ(sample->health().state(), MeasurementHealth::VALID);
  EXPECT_EQ(Assess(*sample).error, SurfaceFixError::kSource);
}

TEST(FakeSurfaceFixTest, EmptyFrameAndNonFinitePositionAreRejected) {
  SurfaceFixTruth empty;
  empty.frame_id = "";
  EXPECT_EQ(Assess(*FakeSurfaceFix().Measure(empty)).error,
            SurfaceFixError::kMissingFrame);
  SurfaceFixTruth nan;
  nan.position_y_m = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(Assess(*FakeSurfaceFix().Measure(nan)).error,
            SurfaceFixError::kNonFinite);
}

}  // namespace
}  // namespace intrinsic::hardware::marine
