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

#include "intrinsic/simulation/gazebo/plugins/altimeter/altimeter_adapter.h"

#include <array>
#include <bit>
#include <charconv>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <functional>
#include <iterator>
#include <limits>
#include <optional>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <vector>

#include "gtest/gtest.h"
#include "intrinsic/embodiment/frame_policy.h"
#include "intrinsic/hardware/marine/altimeter.pb.h"
#include "intrinsic/hardware/marine/altimeter_policy.h"
#include "intrinsic/hardware/marine/fake_altimeter.h"
#include "intrinsic/hardware/marine/measurement_health.pb.h"
#include "intrinsic/hardware/marine/measurement_health_policy.h"

namespace intrinsic::simulation {
namespace {

using ::intrinsic::hardware::marine::AltimeterAssessment;
using ::intrinsic::hardware::marine::AltimeterError;
using ::intrinsic::hardware::marine::AltimeterMeasurementView;
using ::intrinsic::hardware::marine::AltimeterSignedUnitNoise;
using ::intrinsic::hardware::marine::AltimeterTruth;
using ::intrinsic::hardware::marine::AssessAltimeter;
using ::intrinsic::hardware::marine::FakeAltimeter;
using ::intrinsic::hardware::marine::FakeAltimeterConfig;
using ::intrinsic::hardware::marine::kCanonicalMaxRangeM;
using ::intrinsic::hardware::marine::kCanonicalMinRangeM;
using ::intrinsic::hardware::marine::kCanonicalOutOfRangeRangeM;
using ::intrinsic::hardware::marine::MeasurementStateKind;
using ::intrinsic::hardware::marine::SourceHealthView;
using ::intrinsic_proto::hardware::marine::AltimeterMeasurement;
using ::intrinsic_proto::hardware::marine::MeasurementHealth;
using Children = std::vector<std::pair<std::string, std::string>>;
using StateFn = std::function<AltimeterSimulatorState(int)>;

constexpr char kGoldenPath[] =
    "intrinsic/simulation/gazebo/plugins/altimeter/testdata/"
    "altimeter_adapter_seed_42.golden.txt";
constexpr int64_t kStepNanos = 10000000;
constexpr double kSeabedZ = -50.0;

std::string ReadFile(const char* path) {
  std::ifstream file(path, std::ios::binary);
  EXPECT_TRUE(file.is_open()) << path;
  return std::string(std::istreambuf_iterator<char>(file),
                     std::istreambuf_iterator<char>());
}

std::string ToHex(std::string_view bytes) {
  static constexpr char kDigits[] = "0123456789abcdef";
  std::string out(bytes.size() * 2, '\0');
  for (size_t i = 0; i < bytes.size(); ++i) {
    const unsigned char value = static_cast<unsigned char>(bytes[i]);
    out[i * 2] = kDigits[value >> 4];
    out[i * 2 + 1] = kDigits[value & 0x0f];
  }
  return out;
}

std::string IntToString(int64_t value) {
  char buffer[32];
  const std::to_chars_result result =
      std::to_chars(buffer, buffer + sizeof(buffer), value);
  EXPECT_EQ(result.ec, std::errc());
  return std::string(buffer, result.ptr);
}

uint64_t Bits(double value) { return std::bit_cast<uint64_t>(value); }

// ---- SDF construction -----------------------------------------------------

// Contract default world: flat seabed at z = -50.
Children DefaultChildren() { return {{"seabed_z_m", "-50"}}; }

Children With(Children children, const std::string& name,
              const std::string& value) {
  for (auto& child : children) {
    if (child.first == name) {
      child.second = value;
      return children;
    }
  }
  children.emplace_back(name, value);
  return children;
}

Children Without(Children children, const std::string& name) {
  Children kept;
  for (auto& child : children) {
    if (child.first != name) {
      kept.push_back(std::move(child));
    }
  }
  return kept;
}

std::string PluginXml(const Children& children) {
  std::string xml =
      "<plugin filename=\"static://intrinsic::simulation::Altimeter\" "
      "name=\"intrinsic::simulation::Altimeter\">";
  for (const auto& [name, value] : children) {
    xml += "<" + name + ">" + value + "</" + name + ">";
  }
  xml += "</plugin>";
  return xml;
}

std::string Window(int start, int end) {
  return "<start_step>" + IntToString(start) + "</start_step><end_step>" +
         IntToString(end) + "</end_step>";
}

// ---- State and result helpers ---------------------------------------------

// Link z that gives altitude `altitude_m` over the default seabed.
double ZForAltitude(double altitude_m) { return kSeabedZ + altitude_m; }

AltimeterSimulatorState StateAt(int k, double z = -40.0,
                                std::array<double, 4> orientation = {0, 0, 0,
                                                                     1}) {
  AltimeterSimulatorState state;
  state.link_resolved = true;
  state.pose_ready = true;
  state.position_m = {0, 0, z};
  state.orientation_xyzw = orientation;
  SplitAltimeterSimTimeNanos(static_cast<int64_t>(k) * kStepNanos,
                             &state.time_seconds, &state.time_nanos);
  return state;
}

std::vector<AltimeterStepResult> RunSteps(const Children& children, int steps,
                                          const StateFn& state_fn) {
  AltimeterAdapter adapter;
  const AltimeterSimStatus status =
      adapter.ConfigureFromPluginXml(PluginXml(children));
  EXPECT_TRUE(status.ok()) << status.message;
  std::vector<AltimeterStepResult> results;
  for (int k = 0; k < steps; ++k) {
    results.push_back(adapter.Step(state_fn(k)));
  }
  return results;
}

std::vector<AltimeterStepResult> RunSteps(const Children& children, int steps,
                                          double z = -40.0) {
  return RunSteps(children, steps, [z](int k) { return StateAt(k, z); });
}

std::string Bytes(const AltimeterStepResult& result) {
  EXPECT_TRUE(result.measurement.has_value());
  return result.measurement.has_value()
             ? result.measurement->SerializeAsString()
             : std::string();
}

AltimeterMeasurementView ViewOf(const AltimeterMeasurement& sample,
                                std::vector<double>* covariance,
                                std::vector<SourceHealthView>* sources) {
  AltimeterMeasurementView view;
  const MeasurementHealth& health = sample.health();
  view.health.header_present = health.has_header();
  view.health.frame_id = health.header().frame_id();
  view.health.expected_frame_present = true;
  view.health.expected_frame = "body";
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

// Runs the HAL conformance oracle with expected frame "body".
AltimeterAssessment Assess(const AltimeterMeasurement& sample) {
  std::vector<double> covariance;
  std::vector<SourceHealthView> sources;
  return AssessAltimeter(ViewOf(sample, &covariance, &sources));
}

void ExpectOracle(const AltimeterMeasurement& sample, AltimeterError error,
                  MeasurementStateKind state, bool accepted) {
  const AltimeterAssessment assessment = Assess(sample);
  EXPECT_EQ(assessment.error, error);
  EXPECT_EQ(assessment.state, state);
  EXPECT_EQ(assessment.accepted, accepted);
}

void ExpectBoundsAndBeam(const AltimeterMeasurement& sample) {
  ASSERT_TRUE(sample.has_beam_id());
  EXPECT_EQ(sample.beam_id(), "down");
  ASSERT_TRUE(sample.has_min_range_m());
  ASSERT_TRUE(sample.has_max_range_m());
  EXPECT_EQ(Bits(sample.min_range_m()), Bits(0.5));
  EXPECT_EQ(Bits(sample.max_range_m()), Bits(100.0));
}

void ExpectAccepted(const AltimeterMeasurement& sample) {
  ExpectOracle(sample, AltimeterError::kNone, MeasurementStateKind::kValid,
               true);
  EXPECT_EQ(sample.health().state(), MeasurementHealth::VALID);
  ExpectBoundsAndBeam(sample);
  EXPECT_TRUE(sample.has_return());
}

void ExpectNoReturn(const AltimeterMeasurement& sample) {
  EXPECT_TRUE(sample.has_has_return());
  EXPECT_FALSE(sample.has_return());
  EXPECT_FALSE(sample.has_range_m());
  ExpectBoundsAndBeam(sample);
  EXPECT_EQ(sample.health().state(), MeasurementHealth::INVALID);
  ExpectOracle(sample, AltimeterError::kNone, MeasurementStateKind::kInvalid,
               false);
}

void ExpectOutOfRange(const AltimeterMeasurement& sample) {
  ASSERT_TRUE(sample.has_range_m());
  EXPECT_EQ(Bits(sample.range_m()), Bits(kCanonicalOutOfRangeRangeM));
  EXPECT_EQ(Bits(sample.range_m()), Bits(101.0));
  EXPECT_TRUE(sample.has_return());
  EXPECT_EQ(Bits(sample.min_range_m()), Bits(0.5));
  EXPECT_EQ(Bits(sample.max_range_m()), Bits(100.0));
  EXPECT_EQ(sample.health().state(), MeasurementHealth::INVALID);
  ExpectOracle(sample, AltimeterError::kBounds, MeasurementStateKind::kInvalid,
               false);
}

void ExpectDegradedNotAccepted(const AltimeterMeasurement& sample) {
  ExpectOracle(sample, AltimeterError::kNone, MeasurementStateKind::kDegraded,
               false);
  EXPECT_EQ(sample.health().state(), MeasurementHealth::DEGRADED);
}

std::array<int64_t, 2> Stamp(
    const intrinsic_proto::embodiment::StampedHeader& header, bool source) {
  const auto& time = source ? header.source_time() : header.receive_time();
  return {time.seconds(), time.nanos()};
}

int64_t TotalNanos(const std::array<int64_t, 2>& stamp) {
  return stamp[0] * 1000000000LL + stamp[1];
}

// The hand-built fake output the adapter must reproduce byte for byte.
AltimeterMeasurement FakeSample(uint64_t seed, double amplitude, bool no_return,
                                bool out_of_range, int64_t source_nanos,
                                int64_t receive_nanos, double altitude_m) {
  FakeAltimeterConfig config;
  config.seed = seed;
  config.noise_amplitude_m = amplitude;
  config.delay_seconds = 0;
  config.delay_nanos = 0;
  config.no_return = no_return;
  config.out_of_range = out_of_range;
  AltimeterTruth truth;
  truth.frame_id = "body";
  truth.source_seconds = source_nanos / 1000000000;
  truth.source_nanos = static_cast<int32_t>(source_nanos % 1000000000);
  truth.range_present = true;
  truth.range_m = altitude_m;
  truth.beam_present = true;
  truth.beam_id = "down";
  truth.min_present = true;
  truth.min_range_m = kCanonicalMinRangeM;
  truth.max_present = true;
  truth.max_range_m = kCanonicalMaxRangeM;
  truth.has_return_present = true;
  truth.has_return = true;
  truth.quality = 0.75;
  truth.covariance_present = true;
  AltimeterMeasurement sample = *FakeAltimeter(config).Measure(truth);
  auto* receive =
      sample.mutable_health()->mutable_header()->mutable_receive_time();
  receive->set_seconds(receive_nanos / 1000000000);
  receive->set_nanos(static_cast<int32_t>(receive_nanos % 1000000000));
  return sample;
}

// ---- A1: nominal and fake parity ------------------------------------------

TEST(AltimeterAdapterTest, NominalMatchesFakeAltimeterBytes) {
  const std::vector<AltimeterStepResult> results =
      RunSteps(DefaultChildren(), 3, -40.0);
  for (int k = 0; k < 3; ++k) {
    ASSERT_TRUE(results[k].status.ok());
    ASSERT_TRUE(results[k].measurement.has_value());
    EXPECT_EQ(Bytes(results[k]),
              FakeSample(42 + k, 0.0, false, false, k * kStepNanos,
                         k * kStepNanos, 10.0)
                  .SerializeAsString())
        << k;
    const AltimeterMeasurement& sample = *results[k].measurement;
    EXPECT_EQ(Bits(sample.range_m()), Bits(10.0));
    EXPECT_EQ(sample.health().header().frame_id(), "body");
    EXPECT_EQ(sample.health().header().sequence(),
              static_cast<uint64_t>(42 + k));
    ExpectAccepted(sample);
  }
}

TEST(AltimeterAdapterTest, AltitudeIsVerticalAndSignedFromTheSeabed) {
  const std::vector<AltimeterStepResult> shallow =
      RunSteps(With(DefaultChildren(), "seabed_z_m", "-5"), 1, -3.0);
  ASSERT_TRUE(shallow[0].measurement.has_value());
  EXPECT_EQ(Bits(shallow[0].measurement->range_m()), Bits(2.0));
  ExpectAccepted(*shallow[0].measurement);
  // A tilted body does not change the vertical altitude.
  const std::vector<AltimeterStepResult> tilted =
      RunSteps(DefaultChildren(), 1, [](int k) {
        return StateAt(k, -40.0,
                       {0.7071067811865476, 0, 0, 0.7071067811865476});
      });
  ASSERT_TRUE(tilted[0].measurement.has_value());
  EXPECT_EQ(Bits(tilted[0].measurement->range_m()), Bits(10.0));
}

TEST(AltimeterAdapterTest, ClosedIntervalEndpointsAreValidReturns) {
  for (const double altitude : {0.5, 100.0}) {
    const std::vector<AltimeterStepResult> results =
        RunSteps(DefaultChildren(), 2, ZForAltitude(altitude));
    for (const AltimeterStepResult& result : results) {
      ASSERT_TRUE(result.measurement.has_value());
      EXPECT_EQ(Bits(result.measurement->range_m()), Bits(altitude));
      ExpectAccepted(*result.measurement);
    }
  }
}

// ---- A2: no return --------------------------------------------------------

TEST(AltimeterAdapterTest, BlankingZoneAndBelowSeabedGiveNoReturn) {
  for (const double altitude : {0.5 - 1e-6, 0.0, -1.0}) {
    const std::vector<AltimeterStepResult> results =
        RunSteps(DefaultChildren(), 3, ZForAltitude(altitude));
    for (int k = 0; k < 3; ++k) {
      ASSERT_TRUE(results[k].measurement.has_value()) << altitude;
      ExpectNoReturn(*results[k].measurement);
      EXPECT_EQ(Bytes(results[k]),
                FakeSample(42 + k, 0.0, true, false, k * kStepNanos,
                           k * kStepNanos, altitude)
                    .SerializeAsString());
    }
  }
}

TEST(AltimeterAdapterTest, NoiseIsNotAppliedOnNoReturnOrOutOfRange) {
  const Children noisy = With(DefaultChildren(), "noise_amplitude_m", "0.05");
  for (const double altitude : {0.2, -1.0}) {
    for (const AltimeterStepResult& result :
         RunSteps(noisy, 20, ZForAltitude(altitude))) {
      ASSERT_TRUE(result.measurement.has_value());
      ExpectNoReturn(*result.measurement);
    }
  }
  for (const AltimeterStepResult& result :
       RunSteps(noisy, 20, ZForAltitude(100.0 + 1e-6))) {
    ASSERT_TRUE(result.measurement.has_value());
    ExpectOutOfRange(*result.measurement);
  }
}

// ---- A3: out of range -----------------------------------------------------

TEST(AltimeterAdapterTest, BeyondMaxIsOutOfRangeWithTheCanonicalSentinel) {
  const std::vector<AltimeterStepResult> results =
      RunSteps(DefaultChildren(), 3, ZForAltitude(100.0 + 1e-6));
  for (int k = 0; k < 3; ++k) {
    ASSERT_TRUE(results[k].measurement.has_value());
    ExpectOutOfRange(*results[k].measurement);
    EXPECT_EQ(Bytes(results[k]),
              FakeSample(42 + k, 0.0, false, true, k * kStepNanos,
                         k * kStepNanos, 100.0 + 1e-6)
                  .SerializeAsString());
  }
}

TEST(AltimeterAdapterTest, ReturningToRangeIsValidAgainWithoutHysteresis) {
  const std::array<double, 6> altitudes = {10.0,       100.0 + 1e-6, 10.0,
                                           0.5 - 1e-6, 10.0,         0.5};
  const std::vector<AltimeterStepResult> results = RunSteps(
      DefaultChildren(), 6,
      [&altitudes](int k) { return StateAt(k, ZForAltitude(altitudes[k])); });
  for (int k = 0; k < 6; ++k) {
    ASSERT_TRUE(results[k].measurement.has_value()) << k;
    if (k == 1) {
      ExpectOutOfRange(*results[k].measurement);
    } else if (k == 3) {
      ExpectNoReturn(*results[k].measurement);
    } else {
      ExpectAccepted(*results[k].measurement);
    }
  }
}

// ---- A4: noise ------------------------------------------------------------

TEST(AltimeterAdapterTest, NoiseMatchesTheFakeBitExactAndDegrades) {
  constexpr double kAmp = 0.05;
  const Children children =
      With(DefaultChildren(), "noise_amplitude_m", "0.05");
  const std::vector<AltimeterStepResult> results =
      RunSteps(children, 200, -40.0);
  for (int k = 0; k < 200; ++k) {
    ASSERT_TRUE(results[k].measurement.has_value());
    EXPECT_EQ(Bytes(results[k]),
              FakeSample(42 + k, kAmp, false, false, k * kStepNanos,
                         k * kStepNanos, 10.0)
                  .SerializeAsString())
        << k;
    const double noise = kAmp * AltimeterSignedUnitNoise(42 + k);
    EXPECT_DOUBLE_EQ(results[k].measurement->range_m(), 10.0 + noise) << k;
    EXPECT_LE(std::fabs(results[k].measurement->range_m() - 10.0),
              kAmp * (1.0 + 1e-9))
        << k;
    ExpectDegradedNotAccepted(*results[k].measurement);
  }
}

TEST(AltimeterAdapterTest, NoisyRangeBelowMinimumIsNotRepaired) {
  // At a == 0.5 a negative draw puts the range below the 0.5 m minimum. The
  // fake does not repair it, and the oracle reports kBounds.
  const Children children =
      With(DefaultChildren(), "noise_amplitude_m", "0.05");
  const std::vector<AltimeterStepResult> results =
      RunSteps(children, 200, ZForAltitude(0.5));
  int below = 0;
  int at_or_above = 0;
  for (int k = 0; k < 200; ++k) {
    ASSERT_TRUE(results[k].measurement.has_value());
    const AltimeterMeasurement& sample = *results[k].measurement;
    EXPECT_EQ(sample.health().state(), MeasurementHealth::DEGRADED);
    if (sample.range_m() < 0.5) {
      ++below;
      ExpectOracle(sample, AltimeterError::kBounds,
                   MeasurementStateKind::kDegraded, false);
    } else {
      ++at_or_above;
      ExpectDegradedNotAccepted(sample);
    }
  }
  EXPECT_GT(below, 0);
  EXPECT_GT(at_or_above, 0);
}

TEST(AltimeterAdapterTest, NoisyRangeBelowZeroIsNotRepaired) {
  const Children children = With(DefaultChildren(), "noise_amplitude_m", "0.7");
  const std::vector<AltimeterStepResult> results =
      RunSteps(children, 400, ZForAltitude(0.5));
  int negative = 0;
  for (int k = 0; k < 400; ++k) {
    ASSERT_TRUE(results[k].measurement.has_value());
    const AltimeterMeasurement& sample = *results[k].measurement;
    if (sample.range_m() < 0.0) {
      ++negative;
      ExpectOracle(sample, AltimeterError::kRange,
                   MeasurementStateKind::kDegraded, false);
    }
  }
  EXPECT_GT(negative, 0);
}

TEST(AltimeterAdapterTest, NoisyRangeAboveMaximumIsNotRepaired) {
  const Children children =
      With(DefaultChildren(), "noise_amplitude_m", "0.05");
  const std::vector<AltimeterStepResult> results =
      RunSteps(children, 200, ZForAltitude(100.0));
  int above = 0;
  for (int k = 0; k < 200; ++k) {
    ASSERT_TRUE(results[k].measurement.has_value());
    const AltimeterMeasurement& sample = *results[k].measurement;
    if (sample.range_m() > 100.0) {
      ++above;
      ExpectOracle(sample, AltimeterError::kBounds,
                   MeasurementStateKind::kDegraded, false);
    }
  }
  EXPECT_GT(above, 0);
}

// ---- A5: latency ----------------------------------------------------------

TEST(AltimeterAdapterTest, LatencyRampEmitsOriginatedSampleAtLaterReceiveTime) {
  const Children children = With(DefaultChildren(), "latency_steps", "3");
  const std::vector<AltimeterStepResult> results =
      RunSteps(children, 12, [](int k) { return StateAt(k, -40.0 + 0.1 * k); });
  for (int k = 0; k < 12; ++k) {
    ASSERT_TRUE(results[k].status.ok());
    if (k < 3) {
      EXPECT_FALSE(results[k].measurement.has_value()) << k;
      continue;
    }
    ASSERT_TRUE(results[k].measurement.has_value()) << k;
    const AltimeterMeasurement& sample = *results[k].measurement;
    EXPECT_EQ(sample.range_m(), (-40.0 + 0.1 * (k - 3)) - kSeabedZ) << k;
    const auto& header = sample.health().header();
    EXPECT_EQ(TotalNanos(Stamp(header, true)), (k - 3) * kStepNanos);
    EXPECT_EQ(TotalNanos(Stamp(header, false)), k * kStepNanos);
    EXPECT_EQ(
        TotalNanos(Stamp(header, false)) - TotalNanos(Stamp(header, true)),
        3 * kStepNanos);
    EXPECT_EQ(header.sequence(), static_cast<uint64_t>(42 + k - 3));
    ExpectAccepted(sample);
  }
}

TEST(AltimeterAdapterTest, LatencyMatchesFakeWithHandBuiltStamps) {
  const Children children = With(DefaultChildren(), "latency_steps", "2");
  const std::vector<AltimeterStepResult> results = RunSteps(children, 6, -40.0);
  for (int k = 2; k < 6; ++k) {
    EXPECT_EQ(Bytes(results[k]),
              FakeSample(42 + (k - 2), 0.0, false, false, (k - 2) * kStepNanos,
                         k * kStepNanos, 10.0)
                  .SerializeAsString())
        << k;
  }
}

TEST(AltimeterAdapterTest, ZeroLatencyReceiveEqualsSource) {
  const std::vector<AltimeterStepResult> results =
      RunSteps(With(DefaultChildren(), "latency_steps", "0"), 3);
  for (const AltimeterStepResult& result : results) {
    ASSERT_TRUE(result.measurement.has_value());
    const auto& header = result.measurement->health().header();
    EXPECT_EQ(Stamp(header, true), Stamp(header, false));
  }
}

TEST(AltimeterAdapterTest, LatencyKeepsReceiveAtOrAfterSourceAcrossSeconds) {
  const Children children = With(DefaultChildren(), "latency_steps", "4");
  AltimeterAdapter adapter;
  ASSERT_TRUE(adapter.ConfigureFromPluginXml(PluginXml(children)).ok());
  for (int k = 0; k < 10; ++k) {
    AltimeterSimulatorState state = StateAt(0);
    SplitAltimeterSimTimeNanos(static_cast<int64_t>(k) * 300000000,
                               &state.time_seconds, &state.time_nanos);
    const AltimeterStepResult result = adapter.Step(state);
    if (!result.measurement.has_value()) {
      continue;
    }
    const auto& header = result.measurement->health().header();
    EXPECT_GE(TotalNanos(Stamp(header, false)),
              TotalNanos(Stamp(header, true)));
    EXPECT_EQ(
        TotalNanos(Stamp(header, false)) - TotalNanos(Stamp(header, true)),
        4 * 300000000LL);
    ExpectAccepted(*result.measurement);
  }
}

TEST(AltimeterAdapterTest, InvalidLatencyIsInvalidConfig) {
  for (const char* value : {"1001", "-1", "1.5", "abc", "", "2x", "1e2"}) {
    AltimeterAdapter adapter;
    const AltimeterSimStatus status = adapter.ConfigureFromPluginXml(
        PluginXml(With(DefaultChildren(), "latency_steps", value)));
    EXPECT_EQ(status.code, AltimeterSimErrorCode::kInvalidConfig) << value;
  }
  for (const char* value : {"0", "1000", " 7 "}) {
    AltimeterAdapter adapter;
    EXPECT_TRUE(adapter
                    .ConfigureFromPluginXml(PluginXml(
                        With(DefaultChildren(), "latency_steps", value)))
                    .ok())
        << value;
  }
}

// ---- A6: dropout ----------------------------------------------------------

TEST(AltimeterAdapterTest, DropoutWithoutLatencyOmitsWindowAndKeepsLaterDraws) {
  const Children noisy = With(DefaultChildren(), "noise_amplitude_m", "0.05");
  const Children dropped = With(noisy, "dropout_window", Window(5, 8));
  const std::vector<AltimeterStepResult> base = RunSteps(noisy, 12);
  const std::vector<AltimeterStepResult> run = RunSteps(dropped, 12);
  for (int k = 0; k < 12; ++k) {
    ASSERT_TRUE(run[k].status.ok());
    if (k >= 5 && k < 8) {
      EXPECT_FALSE(run[k].measurement.has_value()) << k;
    } else {
      ASSERT_TRUE(run[k].measurement.has_value()) << k;
      EXPECT_EQ(Bytes(run[k]), Bytes(base[k])) << k;
    }
  }
}

TEST(AltimeterAdapterTest, DropoutWithLatencyShiftsTheGapByTheLatency) {
  const Children noisy =
      With(With(DefaultChildren(), "noise_amplitude_m", "0.05"),
           "latency_steps", "2");
  const Children dropped = With(noisy, "dropout_window", Window(5, 8));
  const std::vector<AltimeterStepResult> base = RunSteps(noisy, 14);
  const std::vector<AltimeterStepResult> run = RunSteps(dropped, 14);
  for (int k = 0; k < 14; ++k) {
    const bool absent = k < 2 || (k >= 7 && k < 10);
    ASSERT_EQ(run[k].measurement.has_value(), !absent) << k;
    if (!absent) {
      EXPECT_EQ(Bytes(run[k]), Bytes(base[k])) << k;
    }
  }
}

TEST(AltimeterAdapterTest, DropoutBeatsNoReturnAndOutOfRange) {
  const Children children =
      With(DefaultChildren(), "dropout_window", Window(0, 2));
  for (const double altitude : {0.2, 150.0}) {
    const std::vector<AltimeterStepResult> results =
        RunSteps(children, 4, ZForAltitude(altitude));
    EXPECT_FALSE(results[0].measurement.has_value());
    EXPECT_FALSE(results[1].measurement.has_value());
    EXPECT_TRUE(results[2].measurement.has_value());
    EXPECT_TRUE(results[3].measurement.has_value());
  }
}

TEST(AltimeterAdapterTest, MultipleDropoutWindowsAreHonoured) {
  AltimeterAdapter adapter;
  const std::string xml =
      "<plugin filename=\"static://intrinsic::simulation::Altimeter\" "
      "name=\"intrinsic::simulation::Altimeter\">"
      "<seabed_z_m>-50</seabed_z_m>"
      "<dropout_window>" +
      Window(1, 2) + "</dropout_window><dropout_window>" + Window(4, 6) +
      "</dropout_window></plugin>";
  ASSERT_TRUE(adapter.ConfigureFromPluginXml(xml).ok());
  for (int k = 0; k < 8; ++k) {
    const bool absent = k == 1 || k == 4 || k == 5;
    EXPECT_EQ(adapter.Step(StateAt(k)).measurement.has_value(), !absent) << k;
  }
}

// ---- A7: seed -------------------------------------------------------------

AltimeterMeasurement WithoutSequence(AltimeterMeasurement sample) {
  sample.mutable_health()->mutable_header()->clear_sequence();
  return sample;
}

TEST(AltimeterAdapterTest, SeedWithoutNoiseOnlyChangesTheSequence) {
  const std::vector<AltimeterStepResult> a =
      RunSteps(With(DefaultChildren(), "seed", "42"), 10);
  const std::vector<AltimeterStepResult> b =
      RunSteps(With(DefaultChildren(), "seed", "43"), 10);
  for (int k = 0; k < 10; ++k) {
    ASSERT_TRUE(a[k].measurement.has_value());
    ASSERT_TRUE(b[k].measurement.has_value());
    EXPECT_EQ(a[k].measurement->health().header().sequence(),
              static_cast<uint64_t>(42 + k));
    EXPECT_EQ(b[k].measurement->health().header().sequence(),
              static_cast<uint64_t>(43 + k));
    EXPECT_EQ(WithoutSequence(*a[k].measurement).SerializeAsString(),
              WithoutSequence(*b[k].measurement).SerializeAsString())
        << k;
  }
}

TEST(AltimeterAdapterTest, SeedWithNoiseOnlyChangesRangeAndSequence) {
  const Children base = With(DefaultChildren(), "noise_amplitude_m", "0.05");
  const std::vector<AltimeterStepResult> a =
      RunSteps(With(base, "seed", "42"), 20);
  const std::vector<AltimeterStepResult> b =
      RunSteps(With(base, "seed", "43"), 20);
  int differing = 0;
  for (int k = 0; k < 20; ++k) {
    ASSERT_TRUE(a[k].measurement.has_value());
    ASSERT_TRUE(b[k].measurement.has_value());
    AltimeterMeasurement lhs = WithoutSequence(*a[k].measurement);
    AltimeterMeasurement rhs = WithoutSequence(*b[k].measurement);
    if (lhs.range_m() != rhs.range_m()) {
      ++differing;
    }
    lhs.clear_range_m();
    rhs.clear_range_m();
    EXPECT_EQ(lhs.SerializeAsString(), rhs.SerializeAsString()) << k;
  }
  EXPECT_GT(differing, 0);
}

TEST(AltimeterAdapterTest, MaximumSeedWrapsInsteadOfFailing) {
  const std::vector<AltimeterStepResult> results =
      RunSteps(With(DefaultChildren(), "seed", "18446744073709551615"), 3);
  ASSERT_TRUE(results[2].measurement.has_value());
  EXPECT_EQ(results[0].measurement->health().header().sequence(),
            std::numeric_limits<uint64_t>::max());
  EXPECT_EQ(results[1].measurement->health().header().sequence(), 0u);
  EXPECT_EQ(results[2].measurement->health().header().sequence(), 1u);
}

// ---- A8: config and state robustness --------------------------------------

AltimeterSimStatus ConfigureWith(const Children& children) {
  AltimeterAdapter adapter;
  return adapter.ConfigureFromPluginXml(PluginXml(children));
}

TEST(AltimeterAdapterTest, InvalidConfigIsRejectedAndNeverClamped) {
  const std::vector<std::pair<std::string, std::string>> bad = {
      {"seabed_z_m", "nan"},
      {"seabed_z_m", "inf"},
      {"seabed_z_m", "-inf"},
      {"seabed_z_m", "-50x"},
      {"seabed_z_m", "-50,5"},
      {"seabed_z_m", ""},
      {"noise_amplitude_m", "nan"},
      {"noise_amplitude_m", "inf"},
      {"noise_amplitude_m", "-0.01"},
      {"noise_amplitude_m", "0.05x"},
      {"noise_amplitude_m", ""},
      {"seed", "-1"},
      {"seed", "1.5"},
      {"seed", "abc"},
      {"seed", "18446744073709551616"},
      {"latency_steps", "1001"},
  };
  for (const auto& [name, value] : bad) {
    AltimeterAdapter adapter;
    const AltimeterSimStatus status = adapter.ConfigureFromPluginXml(
        PluginXml(With(DefaultChildren(), name, value)));
    EXPECT_EQ(status.code, AltimeterSimErrorCode::kInvalidConfig)
        << name << "=" << value;
    EXPECT_FALSE(adapter.active());
    for (int k = 0; k < 5; ++k) {
      const AltimeterStepResult result = adapter.Step(StateAt(k));
      EXPECT_EQ(result.status.code, AltimeterSimErrorCode::kInvalidConfig);
      EXPECT_FALSE(result.measurement.has_value());
    }
    EXPECT_EQ(adapter.StepIndexForTesting(), 0) << name << "=" << value;
  }
}

TEST(AltimeterAdapterTest, MissingSeabedIsInvalidConfig) {
  AltimeterAdapter adapter;
  EXPECT_EQ(adapter
                .ConfigureFromPluginXml(
                    PluginXml(Without(DefaultChildren(), "seabed_z_m")))
                .code,
            AltimeterSimErrorCode::kInvalidConfig);
  EXPECT_FALSE(adapter.Step(StateAt(0)).measurement.has_value());
  EXPECT_EQ(ConfigureWith({}).code, AltimeterSimErrorCode::kInvalidConfig);
}

TEST(AltimeterAdapterTest, InvalidDropoutWindowsAreRejected) {
  const std::vector<std::string> bad = {
      Window(5, 4),
      Window(-1, 3),
      "<start_step>1</start_step>",
      "<end_step>1</end_step>",
      "<start_step>1.5</start_step><end_step>3</end_step>",
      "<start_step>1</start_step><end_step>3x</end_step>",
      "<start_step>nan</start_step><end_step>3</end_step>",
  };
  for (const std::string& window : bad) {
    AltimeterAdapter adapter;
    const std::string xml =
        "<plugin filename=\"static://intrinsic::simulation::Altimeter\" "
        "name=\"intrinsic::simulation::Altimeter\">"
        "<seabed_z_m>-50</seabed_z_m><dropout_window>" +
        window + "</dropout_window></plugin>";
    EXPECT_EQ(adapter.ConfigureFromPluginXml(xml).code,
              AltimeterSimErrorCode::kInvalidConfig)
        << window;
    EXPECT_FALSE(adapter.Step(StateAt(0)).measurement.has_value());
  }
}

TEST(AltimeterAdapterTest, MoreThanSixtyFourWindowsAreRejected) {
  for (const int count : {64, 65}) {
    std::string xml =
        "<plugin filename=\"static://intrinsic::simulation::Altimeter\" "
        "name=\"intrinsic::simulation::Altimeter\">"
        "<seabed_z_m>-50</seabed_z_m>";
    for (int i = 0; i < count; ++i) {
      xml +=
          "<dropout_window>" + Window(2 * i, 2 * i + 1) + "</dropout_window>";
    }
    xml += "</plugin>";
    AltimeterAdapter adapter;
    const AltimeterSimStatus status = adapter.ConfigureFromPluginXml(xml);
    if (count == 64) {
      EXPECT_TRUE(status.ok()) << status.message;
    } else {
      EXPECT_EQ(status.code, AltimeterSimErrorCode::kInvalidConfig);
      EXPECT_FALSE(adapter.Step(StateAt(0)).measurement.has_value());
    }
  }
}

TEST(AltimeterAdapterTest, UnknownChildrenAreIgnored) {
  EXPECT_TRUE(ConfigureWith(With(DefaultChildren(), "made_up", "value")).ok());
  EXPECT_TRUE(
      ConfigureWith(With(DefaultChildren(), "min_range_m", "nan")).ok());
  EXPECT_TRUE(ConfigureWith(With(DefaultChildren(), "link_name", "hull")).ok());
}

TEST(AltimeterAdapterTest, RangeBoundsAreNotConfigurable) {
  const std::vector<AltimeterStepResult> results = RunSteps(
      With(With(DefaultChildren(), "min_range_m", "5"), "max_range_m", "20"), 1,
      ZForAltitude(1.0));
  ASSERT_TRUE(results[0].measurement.has_value());
  ExpectAccepted(*results[0].measurement);
}

TEST(AltimeterAdapterTest, UnparseableOrUnconfiguredAdapterFailsClosed) {
  AltimeterAdapter unconfigured;
  EXPECT_FALSE(unconfigured.active());
  const AltimeterStepResult result = unconfigured.Step(StateAt(0));
  EXPECT_EQ(result.status.code, AltimeterSimErrorCode::kInvalidConfig);
  EXPECT_FALSE(result.measurement.has_value());

  AltimeterAdapter garbage;
  EXPECT_EQ(garbage.ConfigureFromPluginXml("<not-a-plugin/>").code,
            AltimeterSimErrorCode::kInvalidConfig);
  EXPECT_FALSE(garbage.Step(StateAt(0)).measurement.has_value());
}

TEST(AltimeterAdapterTest, ReconfigureReplacesAnEarlierBadConfiguration) {
  AltimeterAdapter adapter;
  ASSERT_FALSE(
      adapter.ConfigureFromPluginXml(PluginXml({{"seabed_z_m", "nan"}})).ok());
  ASSERT_TRUE(
      adapter.ConfigureFromPluginXml(PluginXml(DefaultChildren())).ok());
  EXPECT_TRUE(adapter.active());
  EXPECT_TRUE(adapter.Step(StateAt(0)).measurement.has_value());
}

TEST(AltimeterAdapterTest, InvalidStateGivesNoSampleAndStillAdvancesTheStep) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  std::vector<AltimeterSimulatorState> bad;
  AltimeterSimulatorState state = StateAt(1);
  state.link_resolved = false;
  bad.push_back(state);
  state = StateAt(1);
  state.pose_ready = false;
  bad.push_back(state);
  for (const double value : {nan, inf, -inf}) {
    state = StateAt(1);
    state.position_m[2] = value;
    bad.push_back(state);
    state = StateAt(1);
    state.position_m[0] = value;
    bad.push_back(state);
    state = StateAt(1);
    state.orientation_xyzw[1] = value;
    bad.push_back(state);
  }
  state = StateAt(1);
  state.orientation_xyzw = {0, 0, 0, 1.0 + 2e-6};
  bad.push_back(state);
  state = StateAt(1);
  state.orientation_xyzw = {0, 0, 0, 0};
  bad.push_back(state);
  state = StateAt(1);
  state.time_nanos = 1000000000;
  bad.push_back(state);
  state = StateAt(1);
  state.time_nanos = -1;
  bad.push_back(state);

  for (const AltimeterSimulatorState& bad_state : bad) {
    AltimeterAdapter adapter;
    ASSERT_TRUE(
        adapter.ConfigureFromPluginXml(PluginXml(DefaultChildren())).ok());
    EXPECT_TRUE(adapter.Step(StateAt(0)).measurement.has_value());
    const AltimeterStepResult result = adapter.Step(bad_state);
    EXPECT_EQ(result.status.code, AltimeterSimErrorCode::kInvalidState);
    EXPECT_FALSE(result.measurement.has_value());
    EXPECT_EQ(result.step, 1);
    EXPECT_EQ(adapter.StepIndexForTesting(), 2);
    EXPECT_TRUE(adapter.active());
    const AltimeterStepResult recovered = adapter.Step(StateAt(2));
    EXPECT_TRUE(recovered.status.ok());
    ASSERT_TRUE(recovered.measurement.has_value());
    EXPECT_EQ(recovered.step, 2);
    ExpectAccepted(*recovered.measurement);
  }
}

TEST(AltimeterAdapterTest, QuaternionWithinToleranceIsAccepted) {
  AltimeterAdapter adapter;
  ASSERT_TRUE(
      adapter.ConfigureFromPluginXml(PluginXml(DefaultChildren())).ok());
  AltimeterSimulatorState state = StateAt(0);
  state.orientation_xyzw = {0, 0, 0, 1.0 + 5e-7};
  const AltimeterStepResult result = adapter.Step(state);
  EXPECT_TRUE(result.status.ok());
  EXPECT_TRUE(result.measurement.has_value());
}

TEST(AltimeterAdapterTest, NonUnitQuaternionCostsTheDelayedSampleToo) {
  const Children children = With(DefaultChildren(), "latency_steps", "2");
  AltimeterAdapter adapter;
  ASSERT_TRUE(adapter.ConfigureFromPluginXml(PluginXml(children)).ok());
  std::vector<bool> present;
  for (int k = 0; k < 8; ++k) {
    AltimeterSimulatorState state = StateAt(k);
    if (k == 3) {
      state.orientation_xyzw = {0, 0, 0, 1.0 + 2e-6};
    }
    present.push_back(adapter.Step(state).measurement.has_value());
  }
  const std::vector<bool> expected = {false, false, true, false,
                                      true,  false, true, true};
  EXPECT_EQ(present, expected);
}

// ---- Determinism, reset ---------------------------------------------------

TEST(AltimeterAdapterTest, TwoFreshAdaptersAreByteIdenticalOverAThousandSteps) {
  const Children children =
      With(With(DefaultChildren(), "noise_amplitude_m", "0.05"),
           "latency_steps", "5");
  const StateFn state_fn = [](int k) { return StateAt(k, -45.0 + 0.09 * k); };
  const std::vector<AltimeterStepResult> first =
      RunSteps(children, 1000, state_fn);
  const std::vector<AltimeterStepResult> second =
      RunSteps(children, 1000, state_fn);
  for (int k = 0; k < 1000; ++k) {
    ASSERT_EQ(first[k].measurement.has_value(),
              second[k].measurement.has_value());
    if (first[k].measurement.has_value()) {
      EXPECT_EQ(Bytes(first[k]), Bytes(second[k])) << k;
    }
  }
}

TEST(AltimeterAdapterTest, ResetThenRerunReproducesTheSameTrace) {
  const Children children =
      With(With(With(DefaultChildren(), "noise_amplitude_m", "0.05"),
                "latency_steps", "3"),
           "dropout_window", Window(4, 6));
  AltimeterAdapter adapter;
  ASSERT_TRUE(adapter.ConfigureFromPluginXml(PluginXml(children)).ok());
  std::vector<std::optional<std::string>> first;
  for (int k = 0; k < 30; ++k) {
    const AltimeterStepResult result = adapter.Step(StateAt(k));
    first.push_back(result.measurement.has_value()
                        ? std::optional<std::string>(Bytes(result))
                        : std::nullopt);
  }
  EXPECT_EQ(adapter.StepIndexForTesting(), 30);
  adapter.Reset();
  EXPECT_EQ(adapter.StepIndexForTesting(), 0);
  for (int k = 0; k < 30; ++k) {
    const AltimeterStepResult result = adapter.Step(StateAt(k));
    EXPECT_EQ(result.step, k);
    if (first[k].has_value()) {
      ASSERT_TRUE(result.measurement.has_value()) << k;
      EXPECT_EQ(Bytes(result), *first[k]) << k;
    } else {
      EXPECT_FALSE(result.measurement.has_value()) << k;
    }
  }
}

TEST(AltimeterAdapterTest, ResetClearsPendingLatencyQueue) {
  const Children children = With(DefaultChildren(), "latency_steps", "3");
  AltimeterAdapter adapter;
  ASSERT_TRUE(adapter.ConfigureFromPluginXml(PluginXml(children)).ok());
  for (int k = 0; k < 5; ++k) {
    (void)adapter.Step(StateAt(k));
  }
  adapter.Reset();
  for (int k = 0; k < 3; ++k) {
    EXPECT_FALSE(adapter.Step(StateAt(k)).measurement.has_value()) << k;
  }
  EXPECT_TRUE(adapter.Step(StateAt(3)).measurement.has_value());
}

// ---- A9: golden -----------------------------------------------------------

Children GoldenChildren() {
  Children children = DefaultChildren();
  children = With(children, "noise_amplitude_m", "0.05");
  children = With(children, "latency_steps", "1");
  children = With(children, "dropout_window", Window(8, 10));
  children = With(children, "seed", "42");
  return children;
}

std::vector<AltimeterStepResult> GoldenRun() {
  return RunSteps(GoldenChildren(), 20,
                  [](int k) { return StateAt(k, -49.8 + 6.0 * k); });
}

TEST(AltimeterAdapterTest, GoldenSeed42Trace) {
  const std::vector<AltimeterStepResult> results = GoldenRun();
  std::string trace = "altimeter_sim_trace v1\nseed=42\n";
  for (int k = 0; k < 20; ++k) {
    ASSERT_TRUE(results[k].status.ok());
    trace += "step=" + IntToString(k) + " ";
    trace += results[k].measurement.has_value()
                 ? ToHex(results[k].measurement->SerializeAsString())
                 : std::string("absent");
    trace += "\n";
  }
  const std::string golden = ReadFile(kGoldenPath);
  ASSERT_FALSE(golden.empty());
  EXPECT_EQ(trace, golden);
}

TEST(AltimeterAdapterTest, GoldenScenarioShapeIsAsContracted) {
  const std::vector<AltimeterStepResult> results = GoldenRun();
  for (int k = 0; k < 20; ++k) {
    const bool expect_absent = k < 1 || (k >= 9 && k < 11);
    ASSERT_EQ(results[k].measurement.has_value(), !expect_absent) << k;
    if (expect_absent) {
      continue;
    }
    const int origin = k - 1;
    if (origin == 0) {
      ExpectNoReturn(*results[k].measurement);
    } else if (origin >= 17) {
      ExpectOutOfRange(*results[k].measurement);
    } else {
      ExpectDegradedNotAccepted(*results[k].measurement);
      EXPECT_TRUE(results[k].measurement->has_return());
    }
  }
}

}  // namespace
}  // namespace intrinsic::simulation
