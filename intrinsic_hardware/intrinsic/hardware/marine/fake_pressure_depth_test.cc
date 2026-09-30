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

#include "intrinsic/hardware/marine/fake_pressure_depth.h"

#include <vector>

#include "gtest/gtest.h"
#include "intrinsic/embodiment/stamped_header_policy.h"
#include "intrinsic/hardware/marine/measurement_health.pb.h"
#include "intrinsic/hardware/marine/pressure_depth.pb.h"
#include "intrinsic/hardware/marine/pressure_depth_policy.h"

namespace intrinsic::hardware::marine {
namespace {

using intrinsic_proto::hardware::marine::MeasurementHealth;
using intrinsic_proto::hardware::marine::PressureDepthMeasurement;

PressureDepthMeasurementView ViewOf(const PressureDepthMeasurement& sample,
                                    std::vector<double>* covariance,
                                    std::vector<SourceHealthView>* sources) {
  PressureDepthMeasurementView view;
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
  view.pressure_present = sample.has_pressure_pa();
  view.pressure_pa = sample.pressure_pa();
  view.depth_present = sample.has_depth_m();
  view.depth_m = sample.depth_m();
  view.provenance_present = sample.has_depth_provenance();
  view.depth_provenance = sample.depth_provenance();
  view.density_present = sample.has_fluid_density_kg_m3();
  view.fluid_density_kg_m3 = sample.fluid_density_kg_m3();
  return view;
}

PressureDepthAssessment Assess(const PressureDepthMeasurement& sample) {
  std::vector<double> covariance;
  std::vector<SourceHealthView> sources;
  return AssessPressureDepth(ViewOf(sample, &covariance, &sources));
}

TEST(FakePressureDepthTest, FixedSeedTraceIsRepeatable) {
  FakePressureDepth first;
  FakePressureDepth second(FakePressureDepthConfig{});
  const auto left = first.Measure();
  const auto right = second.Measure();
  ASSERT_TRUE(left.has_value());
  ASSERT_TRUE(right.has_value());
  EXPECT_EQ(left->SerializeAsString(), right->SerializeAsString());
  EXPECT_EQ(first.Measure()->SerializeAsString(), left->SerializeAsString());
  EXPECT_TRUE(Assess(*left).accepted);
}

TEST(FakePressureDepthTest, SeedIsTheHeaderSequence) {
  FakePressureDepthConfig config;
  config.seed = 7;
  const auto sample = FakePressureDepth(config).Measure();
  ASSERT_TRUE(sample.has_value());
  EXPECT_EQ(sample->health().header().sequence(), 7u);
  FakePressureDepthConfig other = config;
  other.seed = 8;
  EXPECT_NE(FakePressureDepth(other).Measure()->SerializeAsString(),
            sample->SerializeAsString());
}

TEST(FakePressureDepthTest, BiasIsDegradedAndAdded) {
  FakePressureDepthConfig config;
  config.pressure_bias_pa = 100.0;
  config.depth_bias_m = 0.5;
  PressureDepthTruth truth;
  truth.pressure_pa = 200000.0;
  truth.depth_m = 10.0;
  const auto sample = FakePressureDepth(config).Measure(truth);
  ASSERT_TRUE(sample.has_value());
  EXPECT_DOUBLE_EQ(sample->pressure_pa(), 200100.0);
  EXPECT_DOUBLE_EQ(sample->depth_m(), 10.5);
  EXPECT_EQ(sample->health().state(), MeasurementHealth::DEGRADED);
  EXPECT_EQ(sample->health().header().validity().state(), 1);
  const PressureDepthAssessment assessment = Assess(*sample);
  EXPECT_EQ(assessment.error, PressureDepthError::kNone);
  EXPECT_EQ(assessment.state, MeasurementStateKind::kDegraded);
  EXPECT_FALSE(assessment.accepted);
  EXPECT_EQ(sample->health().state(), MeasurementHealth::DEGRADED);
}

TEST(FakePressureDepthTest, DepthBiasBelowTheSurfaceIsNotRewritten) {
  FakePressureDepthConfig config;
  config.depth_bias_m = -11.0;
  PressureDepthTruth truth;
  truth.depth_m = 10.0;
  const auto sample = FakePressureDepth(config).Measure(truth);
  ASSERT_TRUE(sample.has_value());
  EXPECT_DOUBLE_EQ(sample->depth_m(), -1.0);
  EXPECT_EQ(sample->health().state(), MeasurementHealth::DEGRADED);
  const PressureDepthAssessment assessment = Assess(*sample);
  EXPECT_EQ(assessment.error, PressureDepthError::kDepth);
  EXPECT_EQ(assessment.state, MeasurementStateKind::kDegraded);
  EXPECT_EQ(sample->health().state(), MeasurementHealth::DEGRADED);
}

TEST(FakePressureDepthTest, DelaySetsReceiveTimeAndReversalIsNotRepaired) {
  FakePressureDepthConfig config;
  config.delay_seconds = 2;
  config.delay_nanos = 0;
  PressureDepthTruth truth;
  truth.source_seconds = 100;
  truth.source_nanos = 0;
  const auto delayed = FakePressureDepth(config).Measure(truth);
  ASSERT_TRUE(delayed.has_value());
  EXPECT_EQ(delayed->health().header().receive_time().seconds(), 102);
  EXPECT_EQ(delayed->health().header().receive_time().nanos(), 0);
  EXPECT_EQ(delayed->health().state(), MeasurementHealth::VALID);
  EXPECT_TRUE(Assess(*delayed).accepted);

  config.delay_seconds = -1;
  config.delay_nanos = 0;
  const auto reversed = FakePressureDepth(config).Measure(truth);
  ASSERT_TRUE(reversed.has_value());
  EXPECT_EQ(reversed->health().header().source_time().seconds(), 100);
  EXPECT_EQ(reversed->health().header().receive_time().seconds(), 99);
  EXPECT_EQ(Assess(*reversed).error, PressureDepthError::kTimeReversal);
}

TEST(FakePressureDepthTest, DropoutOmitsTheSample) {
  FakePressureDepthConfig config;
  config.dropout = true;
  config.out_of_range = true;
  EXPECT_FALSE(FakePressureDepth(config).Measure().has_value());
  const PressureDepthAssessment absent =
      AssessPressureDepth(PressureDepthMeasurementView{});
  EXPECT_EQ(absent.error, PressureDepthError::kNone);
  EXPECT_EQ(absent.state, MeasurementStateKind::kAbsent);
  EXPECT_FALSE(absent.accepted);
}

TEST(FakePressureDepthTest, OutOfRangeIsNegativeDepthInvalid) {
  FakePressureDepthConfig config;
  config.out_of_range = true;
  config.pressure_bias_pa = 25.0;
  config.depth_bias_m = 4.0;
  PressureDepthTruth truth;
  truth.pressure_pa = 200000.0;
  truth.depth_m = 10.0;
  const auto sample = FakePressureDepth(config).Measure(truth);
  ASSERT_TRUE(sample.has_value());
  EXPECT_DOUBLE_EQ(sample->depth_m(), kCanonicalOutOfRangeDepthM);
  EXPECT_DOUBLE_EQ(sample->pressure_pa(), 200025.0);
  EXPECT_EQ(sample->depth_provenance(),
            PressureDepthMeasurement::DEPTH_PROVENANCE_FROM_PRESSURE);
  EXPECT_DOUBLE_EQ(sample->fluid_density_kg_m3(), 1025.0);
  EXPECT_EQ(sample->health().state(), MeasurementHealth::INVALID);
  const PressureDepthAssessment assessment = Assess(*sample);
  EXPECT_EQ(assessment.error, PressureDepthError::kDepth);
  EXPECT_EQ(assessment.state, MeasurementStateKind::kInvalid);
  EXPECT_FALSE(assessment.accepted);
  EXPECT_EQ(sample->health().state(), MeasurementHealth::INVALID);
}

TEST(FakePressureDepthTest, PressureOnlyAndDirectDepth) {
  PressureDepthTruth pressure_only;
  pressure_only.depth_present = false;
  pressure_only.provenance_present = false;
  pressure_only.density_present = false;
  const auto pressure = FakePressureDepth().Measure(pressure_only);
  ASSERT_TRUE(pressure.has_value());
  EXPECT_TRUE(pressure->has_pressure_pa());
  EXPECT_FALSE(pressure->has_depth_m());
  EXPECT_FALSE(pressure->has_depth_provenance());
  EXPECT_FALSE(pressure->has_fluid_density_kg_m3());
  EXPECT_TRUE(Assess(*pressure).accepted);

  PressureDepthTruth direct;
  direct.pressure_present = false;
  direct.depth_provenance = 1;
  direct.density_present = false;
  const auto depth = FakePressureDepth().Measure(direct);
  ASSERT_TRUE(depth.has_value());
  EXPECT_FALSE(depth->has_pressure_pa());
  EXPECT_DOUBLE_EQ(depth->depth_m(), 10.0);
  EXPECT_EQ(depth->depth_provenance(),
            PressureDepthMeasurement::DEPTH_PROVENANCE_DIRECT);
  EXPECT_TRUE(Assess(*depth).accepted);
}

}  // namespace
}  // namespace intrinsic::hardware::marine
