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

#include "intrinsic/hardware/marine/fake_altimeter.h"

#include <vector>

#include "gtest/gtest.h"
#include "intrinsic/embodiment/stamped_header_policy.h"
#include "intrinsic/hardware/marine/altimeter.pb.h"
#include "intrinsic/hardware/marine/altimeter_policy.h"
#include "intrinsic/hardware/marine/measurement_health.pb.h"

namespace intrinsic::hardware::marine {
namespace {

using intrinsic_proto::hardware::marine::AltimeterMeasurement;
using intrinsic_proto::hardware::marine::MeasurementHealth;

AltimeterMeasurementView ViewOf(const AltimeterMeasurement& sample,
                                std::vector<double>* covariance,
                                std::vector<SourceHealthView>* sources) {
  AltimeterMeasurementView view;
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
  view.range_present = sample.has_range_m();
  view.range_m = sample.range_m();
  view.beam_present = sample.has_beam_id();
  view.beam_id = sample.beam_id();
  view.min_present = sample.has_min_range_m();
  view.min_range_m = sample.min_range_m();
  view.max_present = sample.has_max_range_m();
  view.max_range_m = sample.max_range_m();
  view.has_return_present = sample.has_has_return();
  view.has_return = sample.has_return();
  return view;
}

AltimeterAssessment Assess(const AltimeterMeasurement& sample) {
  std::vector<double> covariance;
  std::vector<SourceHealthView> sources;
  return AssessAltimeter(ViewOf(sample, &covariance, &sources));
}

TEST(FakeAltimeterTest, FixedSeedTraceIsRepeatable) {
  FakeAltimeter first;
  FakeAltimeter second(FakeAltimeterConfig{});
  const auto left = first.Measure();
  const auto right = second.Measure();
  ASSERT_TRUE(left.has_value());
  ASSERT_TRUE(right.has_value());
  EXPECT_EQ(left->SerializeAsString(), right->SerializeAsString());
  EXPECT_EQ(first.Measure()->SerializeAsString(), left->SerializeAsString());
  EXPECT_TRUE(Assess(*left).accepted);
  EXPECT_EQ(AltimeterMeasurement::descriptor()->FindFieldByName("altitude_m"),
            nullptr);
}

TEST(FakeAltimeterTest, SeedIsTheHeaderSequenceAndTheNoiseMix) {
  FakeAltimeterConfig config;
  config.seed = 7;
  config.noise_amplitude_m = 0.1;
  const auto sample = FakeAltimeter(config).Measure();
  ASSERT_TRUE(sample.has_value());
  EXPECT_EQ(sample->health().header().sequence(), 7u);
  EXPECT_DOUBLE_EQ(sample->range_m(), 10.0 + 0.1 * AltimeterSignedUnitNoise(7));
  FakeAltimeterConfig other = config;
  other.seed = 8;
  const auto shifted = FakeAltimeter(other).Measure();
  ASSERT_TRUE(shifted.has_value());
  EXPECT_NE(shifted->SerializeAsString(), sample->SerializeAsString());
  EXPECT_NE(shifted->range_m(), sample->range_m());
}

TEST(FakeAltimeterTest, NoiseIsDegradedAndNotRepaired) {
  EXPECT_DOUBLE_EQ(AltimeterSignedUnitNoise(42), 0.4831297575436466);
  FakeAltimeterConfig config;
  config.noise_amplitude_m = 0.1;
  const auto sample = FakeAltimeter(config).Measure();
  ASSERT_TRUE(sample.has_value());
  EXPECT_DOUBLE_EQ(sample->range_m(), 10.0 + 0.1 * 0.4831297575436466);
  EXPECT_EQ(sample->health().state(), MeasurementHealth::DEGRADED);
  EXPECT_EQ(sample->health().header().validity().state(), 1);
  const AltimeterAssessment assessment = Assess(*sample);
  EXPECT_EQ(assessment.error, AltimeterError::kNone);
  EXPECT_EQ(assessment.state, MeasurementStateKind::kDegraded);
  EXPECT_FALSE(assessment.accepted);
  EXPECT_EQ(sample->health().state(), MeasurementHealth::DEGRADED);

  config.noise_amplitude_m = -30.0;
  const auto negative = FakeAltimeter(config).Measure();
  ASSERT_TRUE(negative.has_value());
  EXPECT_LT(negative->range_m(), 0.0);
  EXPECT_EQ(negative->health().state(), MeasurementHealth::DEGRADED);
  const AltimeterAssessment rejected = Assess(*negative);
  EXPECT_EQ(rejected.error, AltimeterError::kRange);
  EXPECT_EQ(rejected.state, MeasurementStateKind::kDegraded);
  EXPECT_EQ(negative->health().state(), MeasurementHealth::DEGRADED);
}

TEST(FakeAltimeterTest, DelaySetsReceiveTimeAndReversalIsNotRepaired) {
  FakeAltimeterConfig config;
  config.delay_seconds = 2;
  config.delay_nanos = 0;
  AltimeterTruth truth;
  truth.source_seconds = 100;
  truth.source_nanos = 0;
  const auto delayed = FakeAltimeter(config).Measure(truth);
  ASSERT_TRUE(delayed.has_value());
  EXPECT_EQ(delayed->health().header().receive_time().seconds(), 102);
  EXPECT_EQ(delayed->health().header().receive_time().nanos(), 0);
  EXPECT_EQ(delayed->health().state(), MeasurementHealth::VALID);
  EXPECT_TRUE(Assess(*delayed).accepted);

  config.delay_seconds = -1;
  config.delay_nanos = 0;
  const auto reversed = FakeAltimeter(config).Measure(truth);
  ASSERT_TRUE(reversed.has_value());
  EXPECT_EQ(reversed->health().header().source_time().seconds(), 100);
  EXPECT_EQ(reversed->health().header().receive_time().seconds(), 99);
  EXPECT_EQ(Assess(*reversed).error, AltimeterError::kTimeReversal);
}

TEST(FakeAltimeterTest, DropoutOmitsTheSample) {
  FakeAltimeterConfig config;
  config.dropout = true;
  config.no_return = true;
  config.out_of_range = true;
  EXPECT_FALSE(FakeAltimeter(config).Measure().has_value());
  const AltimeterAssessment absent =
      AssessAltimeter(AltimeterMeasurementView{});
  EXPECT_EQ(absent.error, AltimeterError::kNone);
  EXPECT_EQ(absent.state, MeasurementStateKind::kAbsent);
  EXPECT_FALSE(absent.accepted);
}

TEST(FakeAltimeterTest, NoReturnOmitsRangeAndIsInvalid) {
  FakeAltimeterConfig config;
  config.no_return = true;
  config.out_of_range = true;
  config.noise_amplitude_m = 4.0;
  const auto sample = FakeAltimeter(config).Measure();
  ASSERT_TRUE(sample.has_value());
  EXPECT_FALSE(sample->has_range_m());
  EXPECT_TRUE(sample->has_has_return());
  EXPECT_FALSE(sample->has_return());
  EXPECT_EQ(sample->beam_id(), "down");
  EXPECT_DOUBLE_EQ(sample->min_range_m(), 0.5);
  EXPECT_DOUBLE_EQ(sample->max_range_m(), 100.0);
  EXPECT_EQ(sample->health().state(), MeasurementHealth::INVALID);
  const AltimeterAssessment assessment = Assess(*sample);
  EXPECT_EQ(assessment.error, AltimeterError::kNone);
  EXPECT_EQ(assessment.state, MeasurementStateKind::kInvalid);
  EXPECT_FALSE(assessment.accepted);
  EXPECT_EQ(sample->health().state(), MeasurementHealth::INVALID);
}

TEST(FakeAltimeterTest, OutOfRangeIsCanonicalFixtureInvalid) {
  FakeAltimeterConfig config;
  config.out_of_range = true;
  config.noise_amplitude_m = 4.0;
  AltimeterTruth truth;
  truth.range_m = 10.0;
  truth.min_range_m = 1.0;
  truth.max_range_m = 20.0;
  const auto sample = FakeAltimeter(config).Measure(truth);
  ASSERT_TRUE(sample.has_value());
  EXPECT_DOUBLE_EQ(sample->range_m(), kCanonicalOutOfRangeRangeM);
  EXPECT_DOUBLE_EQ(sample->min_range_m(), kCanonicalMinRangeM);
  EXPECT_DOUBLE_EQ(sample->max_range_m(), kCanonicalMaxRangeM);
  EXPECT_TRUE(sample->has_return());
  EXPECT_EQ(sample->beam_id(), "down");
  EXPECT_EQ(sample->health().state(), MeasurementHealth::INVALID);
  const AltimeterAssessment assessment = Assess(*sample);
  EXPECT_EQ(assessment.error, AltimeterError::kBounds);
  EXPECT_EQ(assessment.state, MeasurementStateKind::kInvalid);
  EXPECT_FALSE(assessment.accepted);
  EXPECT_EQ(sample->health().state(), MeasurementHealth::INVALID);
}

TEST(FakeAltimeterTest, InvalidQualityIsNotRewritten) {
  AltimeterTruth truth;
  truth.quality = 1.5;
  const auto sample = FakeAltimeter().Measure(truth);
  ASSERT_TRUE(sample.has_value());
  EXPECT_FLOAT_EQ(sample->health().quality(), 1.5f);
  EXPECT_EQ(sample->health().state(), MeasurementHealth::VALID);
  EXPECT_EQ(Assess(*sample).error, AltimeterError::kQuality);
  EXPECT_EQ(sample->health().state(), MeasurementHealth::VALID);
}

TEST(FakeAltimeterTest, RangeOnlyTruthIsAccepted) {
  AltimeterTruth truth;
  truth.has_return_present = false;
  truth.beam_present = false;
  const auto sample = FakeAltimeter().Measure(truth);
  ASSERT_TRUE(sample.has_value());
  EXPECT_TRUE(sample->has_range_m());
  EXPECT_FALSE(sample->has_has_return());
  EXPECT_FALSE(sample->has_beam_id());
  EXPECT_DOUBLE_EQ(sample->range_m(), 10.0);
  EXPECT_TRUE(Assess(*sample).accepted);
}

}  // namespace
}  // namespace intrinsic::hardware::marine
