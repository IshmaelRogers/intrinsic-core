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

#include "intrinsic/simulation/gazebo/plugins/depth/depth_adapter.h"

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
#include "intrinsic/hardware/marine/fake_imu.h"
#include "intrinsic/hardware/marine/fake_pressure_depth.h"
#include "intrinsic/hardware/marine/measurement_health.pb.h"
#include "intrinsic/hardware/marine/measurement_health_policy.h"
#include "intrinsic/hardware/marine/pressure_depth.pb.h"
#include "intrinsic/hardware/marine/pressure_depth_policy.h"

namespace intrinsic::simulation {
namespace {

using ::intrinsic::hardware::marine::AssessPressureDepth;
using ::intrinsic::hardware::marine::DepthProvenanceKind;
using ::intrinsic::hardware::marine::FakePressureDepth;
using ::intrinsic::hardware::marine::FakePressureDepthConfig;
using ::intrinsic::hardware::marine::ImuSignedUnitNoise;
using ::intrinsic::hardware::marine::kCanonicalOutOfRangeDepthM;
using ::intrinsic::hardware::marine::MeasurementStateKind;
using ::intrinsic::hardware::marine::PressureDepthAssessment;
using ::intrinsic::hardware::marine::PressureDepthError;
using ::intrinsic::hardware::marine::PressureDepthMeasurementView;
using ::intrinsic::hardware::marine::PressureDepthTruth;
using ::intrinsic::hardware::marine::SourceHealthView;
using ::intrinsic_proto::hardware::marine::MeasurementHealth;
using ::intrinsic_proto::hardware::marine::PressureDepthMeasurement;
using Children = std::vector<std::pair<std::string, std::string>>;
using StateFn = std::function<DepthSimulatorState(int)>;

constexpr char kGoldenPath[] =
    "intrinsic/simulation/gazebo/plugins/depth/testdata/"
    "depth_adapter_seed_42.golden.txt";
constexpr int64_t kStepNanos = 10000000;
constexpr uint64_t kGamma = 0x9E3779B97F4A7C15ULL;

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

// Contract default world: surface at z = 0, no rating limit.
Children DefaultChildren() { return {}; }

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

std::string PluginXml(const Children& children) {
  std::string xml =
      "<plugin filename=\"static://intrinsic::simulation::Depth\" "
      "name=\"intrinsic::simulation::Depth\">";
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

DepthSimulatorState StateAt(int k, double z = -10.0,
                            std::array<double, 4> orientation = {0, 0, 0, 1}) {
  DepthSimulatorState state;
  state.link_resolved = true;
  state.pose_ready = true;
  state.position_m = {0, 0, z};
  state.orientation_xyzw = orientation;
  SplitDepthSimTimeNanos(static_cast<int64_t>(k) * kStepNanos,
                         &state.time_seconds, &state.time_nanos);
  return state;
}

std::vector<DepthStepResult> RunSteps(const Children& children, int steps,
                                      const StateFn& state_fn) {
  DepthAdapter adapter;
  const DepthSimStatus status =
      adapter.ConfigureFromPluginXml(PluginXml(children));
  EXPECT_TRUE(status.ok()) << status.message;
  std::vector<DepthStepResult> results;
  for (int k = 0; k < steps; ++k) {
    results.push_back(adapter.Step(state_fn(k)));
  }
  return results;
}

std::vector<DepthStepResult> RunSteps(const Children& children, int steps,
                                      double z = -10.0) {
  return RunSteps(children, steps, [z](int k) { return StateAt(k, z); });
}

std::string Bytes(const DepthStepResult& result) {
  EXPECT_TRUE(result.measurement.has_value());
  return result.measurement.has_value()
             ? result.measurement->SerializeAsString()
             : std::string();
}

PressureDepthMeasurementView ViewOf(const PressureDepthMeasurement& sample,
                                    std::vector<double>* covariance,
                                    std::vector<SourceHealthView>* sources) {
  PressureDepthMeasurementView view;
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
  view.pressure_present = sample.has_pressure_pa();
  view.pressure_pa = sample.pressure_pa();
  view.depth_present = sample.has_depth_m();
  view.depth_m = sample.depth_m();
  view.provenance_present = sample.has_depth_provenance();
  view.depth_provenance = static_cast<int>(sample.depth_provenance());
  view.density_present = sample.has_fluid_density_kg_m3();
  view.fluid_density_kg_m3 = sample.fluid_density_kg_m3();
  return view;
}

// Runs the HAL conformance oracle with expected frame "body".
PressureDepthAssessment Assess(const PressureDepthMeasurement& sample) {
  std::vector<double> covariance;
  std::vector<SourceHealthView> sources;
  return AssessPressureDepth(ViewOf(sample, &covariance, &sources));
}

void ExpectOracle(const PressureDepthMeasurement& sample,
                  PressureDepthError error, MeasurementStateKind state,
                  bool accepted) {
  const PressureDepthAssessment assessment = Assess(sample);
  EXPECT_EQ(assessment.error, error);
  EXPECT_EQ(assessment.state, state);
  EXPECT_EQ(assessment.accepted, accepted);
}

void ExpectAccepted(const PressureDepthMeasurement& sample) {
  ExpectOracle(sample, PressureDepthError::kNone, MeasurementStateKind::kValid,
               true);
  EXPECT_EQ(sample.health().state(), MeasurementHealth::VALID);
  EXPECT_EQ(Assess(sample).provenance, DepthProvenanceKind::kDirect);
}

void ExpectDegradedNotAccepted(const PressureDepthMeasurement& sample) {
  ExpectOracle(sample, PressureDepthError::kNone,
               MeasurementStateKind::kDegraded, false);
  EXPECT_EQ(sample.health().state(), MeasurementHealth::DEGRADED);
}

void ExpectOutOfRange(const PressureDepthMeasurement& sample) {
  ASSERT_TRUE(sample.has_depth_m());
  EXPECT_EQ(Bits(sample.depth_m()), Bits(kCanonicalOutOfRangeDepthM));
  EXPECT_EQ(sample.health().state(), MeasurementHealth::INVALID);
  ExpectOracle(sample, PressureDepthError::kDepth,
               MeasurementStateKind::kInvalid, false);
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
PressureDepthMeasurement FakeSample(uint64_t seed, double depth_bias_m,
                                    bool out_of_range, int64_t source_nanos,
                                    int64_t receive_nanos, double depth_m) {
  FakePressureDepthConfig config;
  config.seed = seed;
  config.depth_bias_m = depth_bias_m;
  config.delay_seconds = 0;
  config.delay_nanos = 0;
  config.out_of_range = out_of_range;
  PressureDepthTruth truth;
  truth.frame_id = "body";
  truth.source_seconds = source_nanos / 1000000000;
  truth.source_nanos = static_cast<int32_t>(source_nanos % 1000000000);
  truth.pressure_present = false;
  truth.depth_present = true;
  truth.depth_m = depth_m;
  truth.provenance_present = true;
  truth.depth_provenance = 1;
  truth.density_present = false;
  truth.quality = 0.75;
  truth.covariance_present = true;
  PressureDepthMeasurement sample = *FakePressureDepth(config).Measure(truth);
  auto* receive =
      sample.mutable_health()->mutable_header()->mutable_receive_time();
  receive->set_seconds(receive_nanos / 1000000000);
  receive->set_nanos(static_cast<int32_t>(receive_nanos % 1000000000));
  return sample;
}

// ---- Independent noise formula (does not call the adapter) ----------------

double Unit(uint64_t seed, int k) {
  return ImuSignedUnitNoise(seed + (2 * static_cast<uint64_t>(k)) * kGamma);
}

// ---- D1: nominal and fake parity ------------------------------------------

TEST(DepthAdapterTest, NominalMatchesFakePressureDepthBytes) {
  const std::vector<DepthStepResult> results =
      RunSteps(DefaultChildren(), 3, -10.0);
  for (int k = 0; k < 3; ++k) {
    ASSERT_TRUE(results[k].status.ok());
    ASSERT_TRUE(results[k].measurement.has_value());
    const PressureDepthMeasurement expected =
        FakeSample(42 + k, 0.0, false, k * kStepNanos, k * kStepNanos, 10.0);
    EXPECT_EQ(Bytes(results[k]), expected.SerializeAsString()) << k;
    const PressureDepthMeasurement& sample = *results[k].measurement;
    EXPECT_EQ(Bits(sample.depth_m()), Bits(10.0));
    EXPECT_FALSE(sample.has_pressure_pa());
    EXPECT_FALSE(sample.has_fluid_density_kg_m3());
    ASSERT_TRUE(sample.has_depth_provenance());
    EXPECT_EQ(sample.depth_provenance(),
              PressureDepthMeasurement::DEPTH_PROVENANCE_DIRECT);
    EXPECT_EQ(sample.health().header().frame_id(), "body");
    EXPECT_EQ(sample.health().header().sequence(),
              static_cast<uint64_t>(42 + k));
    ExpectAccepted(sample);
  }
}

TEST(DepthAdapterTest, SurfaceOffsetFlipsTheSignConvention) {
  const std::vector<DepthStepResult> results =
      RunSteps(With(DefaultChildren(), "surface_z_m", "2"), 2, -3.0);
  for (const DepthStepResult& result : results) {
    ASSERT_TRUE(result.measurement.has_value());
    EXPECT_EQ(Bits(result.measurement->depth_m()), Bits(5.0));
    ExpectAccepted(*result.measurement);
  }
  // Deeper means lower z: a link below the surface plane has positive depth.
  const std::vector<DepthStepResult> deeper =
      RunSteps(With(DefaultChildren(), "surface_z_m", "2"), 1, -4.0);
  EXPECT_EQ(Bits(deeper[0].measurement->depth_m()), Bits(6.0));
}

TEST(DepthAdapterTest, PressureIsNeverSynthesized) {
  const std::vector<DepthStepResult> results =
      RunSteps(With(With(DefaultChildren(), "noise_amplitude_m", "0.05"),
                    "bias_m", "0.1"),
               50, -30.0);
  for (const DepthStepResult& result : results) {
    ASSERT_TRUE(result.measurement.has_value());
    EXPECT_FALSE(result.measurement->has_pressure_pa());
    EXPECT_FALSE(result.measurement->has_fluid_density_kg_m3());
    EXPECT_EQ(result.measurement->depth_provenance(),
              PressureDepthMeasurement::DEPTH_PROVENANCE_DIRECT);
  }
}

// ---- D2: bias -------------------------------------------------------------

TEST(DepthAdapterTest, BiasIsAddedBitExactAndDegrades) {
  for (const std::string bias : {"0.2", "-0.3"}) {
    const double bias_value = bias == "0.2" ? 0.2 : -0.3;
    const std::vector<DepthStepResult> results =
        RunSteps(With(DefaultChildren(), "bias_m", bias), 4, -10.0);
    for (int k = 0; k < 4; ++k) {
      ASSERT_TRUE(results[k].measurement.has_value());
      const PressureDepthMeasurement& sample = *results[k].measurement;
      EXPECT_EQ(Bits(sample.depth_m()), Bits(10.0 + bias_value)) << bias;
      ExpectDegradedNotAccepted(sample);
      EXPECT_EQ(Bytes(results[k]),
                FakeSample(42 + k, bias_value, false, k * kStepNanos,
                           k * kStepNanos, 10.0)
                    .SerializeAsString());
    }
  }
}

// ---- D3: noise ------------------------------------------------------------

TEST(DepthAdapterTest, NoiseMatchesTheContractFormulaBitExact) {
  constexpr double kAmp = 0.05;
  for (const double bias : {0.0, 0.01}) {
    const Children children =
        With(With(DefaultChildren(), "noise_amplitude_m", "0.05"), "bias_m",
             bias == 0.0 ? "0" : "0.01");
    const std::vector<DepthStepResult> results = RunSteps(children, 200, -10.0);
    for (int k = 0; k < 200; ++k) {
      ASSERT_TRUE(results[k].measurement.has_value());
      const double effective = std::fma(kAmp, Unit(42, k), bias);
      EXPECT_EQ(Bits(results[k].measurement->depth_m()), Bits(10.0 + effective))
          << k;
      EXPECT_LE(std::fabs(effective - bias), kAmp * (1.0 + 1e-12)) << k;
      ExpectDegradedNotAccepted(*results[k].measurement);
    }
  }
}

TEST(DepthAdapterTest, NoiseOnlyNeverYieldsAValidInRangeSample) {
  const Children children =
      With(With(DefaultChildren(), "noise_amplitude_m", "0.05"), "bias_m", "0");
  const std::vector<DepthStepResult> results = RunSteps(children, 1000, -10.0);
  for (int k = 0; k < 1000; ++k) {
    ASSERT_TRUE(results[k].measurement.has_value());
    EXPECT_NE(results[k].measurement->health().state(),
              MeasurementHealth::VALID)
        << k;
    ExpectDegradedNotAccepted(*results[k].measurement);
  }
}

TEST(DepthAdapterTest, ZeroEffectiveBiasWithNoiseIsForcedDegraded) {
  // The smallest denormal amplitude underflows fma(amp, u, 0) to exactly 0
  // whenever |u| < 0.5, so the fake alone would return VALID.
  const Children children = With(
      With(DefaultChildren(), "noise_amplitude_m", "5e-324"), "bias_m", "0");
  constexpr double kAmp = 5e-324;
  const std::vector<DepthStepResult> results = RunSteps(children, 200, -10.0);
  int zero_bias_steps = 0;
  for (int k = 0; k < 200; ++k) {
    ASSERT_TRUE(results[k].measurement.has_value());
    if (std::fma(kAmp, Unit(42, k), 0.0) == 0.0) {
      ++zero_bias_steps;
      EXPECT_EQ(
          FakeSample(42 + k, 0.0, false, k * kStepNanos, k * kStepNanos, 10.0)
              .health()
              .state(),
          MeasurementHealth::VALID);
    }
    ExpectDegradedNotAccepted(*results[k].measurement);
  }
  EXPECT_GT(zero_bias_steps, 0);
}

// ---- D4: latency ----------------------------------------------------------

TEST(DepthAdapterTest, LatencyRampEmitsOriginatedSampleAtLaterReceiveTime) {
  const Children children = With(DefaultChildren(), "latency_steps", "3");
  const std::vector<DepthStepResult> results =
      RunSteps(children, 12, [](int k) { return StateAt(k, -10.0 - 0.1 * k); });
  for (int k = 0; k < 12; ++k) {
    ASSERT_TRUE(results[k].status.ok());
    if (k < 3) {
      EXPECT_FALSE(results[k].measurement.has_value()) << k;
      continue;
    }
    ASSERT_TRUE(results[k].measurement.has_value()) << k;
    const PressureDepthMeasurement& sample = *results[k].measurement;
    EXPECT_EQ(sample.depth_m(), 10.0 + 0.1 * (k - 3)) << k;
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

TEST(DepthAdapterTest, LatencyMatchesFakeWithHandBuiltStamps) {
  const Children children = With(DefaultChildren(), "latency_steps", "2");
  const std::vector<DepthStepResult> results = RunSteps(children, 6, -10.0);
  for (int k = 2; k < 6; ++k) {
    EXPECT_EQ(Bytes(results[k]),
              FakeSample(42 + (k - 2), 0.0, false, (k - 2) * kStepNanos,
                         k * kStepNanos, 10.0)
                  .SerializeAsString())
        << k;
  }
}

TEST(DepthAdapterTest, ZeroLatencyReceiveEqualsSource) {
  const std::vector<DepthStepResult> results =
      RunSteps(With(DefaultChildren(), "latency_steps", "0"), 3);
  for (const DepthStepResult& result : results) {
    ASSERT_TRUE(result.measurement.has_value());
    const auto& header = result.measurement->health().header();
    EXPECT_EQ(Stamp(header, true), Stamp(header, false));
  }
}

TEST(DepthAdapterTest, LatencyKeepsReceiveAtOrAfterSourceAcrossSecondBoundary) {
  const Children children = With(DefaultChildren(), "latency_steps", "4");
  DepthAdapter adapter;
  ASSERT_TRUE(adapter.ConfigureFromPluginXml(PluginXml(children)).ok());
  // dt = 0.3 s so the stamps cross whole seconds.
  for (int k = 0; k < 10; ++k) {
    DepthSimulatorState state = StateAt(0);
    SplitDepthSimTimeNanos(static_cast<int64_t>(k) * 300000000,
                           &state.time_seconds, &state.time_nanos);
    const DepthStepResult result = adapter.Step(state);
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

TEST(DepthAdapterTest, InvalidLatencyIsInvalidConfig) {
  for (const char* value : {"1001", "-1", "1.5", "abc", "", "2x", "1e2"}) {
    DepthAdapter adapter;
    const DepthSimStatus status = adapter.ConfigureFromPluginXml(
        PluginXml(With(DefaultChildren(), "latency_steps", value)));
    EXPECT_EQ(status.code, DepthSimErrorCode::kInvalidConfig) << value;
  }
  for (const char* value : {"0", "1000", " 7 "}) {
    DepthAdapter adapter;
    EXPECT_TRUE(adapter
                    .ConfigureFromPluginXml(PluginXml(
                        With(DefaultChildren(), "latency_steps", value)))
                    .ok())
        << value;
  }
}

// ---- D5: dropout ----------------------------------------------------------

TEST(DepthAdapterTest, DropoutWithoutLatencyOmitsWindowAndKeepsLaterDraws) {
  const Children noisy = With(DefaultChildren(), "noise_amplitude_m", "0.05");
  const Children dropped = With(noisy, "dropout_window", Window(5, 8));
  const std::vector<DepthStepResult> base = RunSteps(noisy, 12);
  const std::vector<DepthStepResult> run = RunSteps(dropped, 12);
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

TEST(DepthAdapterTest, DropoutWithLatencyShiftsTheGapByTheLatency) {
  const Children noisy =
      With(With(DefaultChildren(), "noise_amplitude_m", "0.05"),
           "latency_steps", "2");
  const Children dropped = With(noisy, "dropout_window", Window(5, 8));
  const std::vector<DepthStepResult> base = RunSteps(noisy, 14);
  const std::vector<DepthStepResult> run = RunSteps(dropped, 14);
  for (int k = 0; k < 14; ++k) {
    const bool absent = k < 2 || (k >= 7 && k < 10);
    ASSERT_EQ(run[k].measurement.has_value(), !absent) << k;
    if (!absent) {
      EXPECT_EQ(Bytes(run[k]), Bytes(base[k])) << k;
    }
  }
}

TEST(DepthAdapterTest, DropoutWindowsAreHalfOpenAndMayBeEmpty) {
  const Children children =
      With(With(DefaultChildren(), "dropout_window", Window(2, 2)),
           "latency_steps", "0");
  const std::vector<DepthStepResult> run = RunSteps(children, 5);
  for (const DepthStepResult& result : run) {
    EXPECT_TRUE(result.measurement.has_value());
  }
}

TEST(DepthAdapterTest, MultipleDropoutWindowsAreHonoured) {
  DepthAdapter adapter;
  const std::string xml =
      "<plugin filename=\"static://intrinsic::simulation::Depth\" "
      "name=\"intrinsic::simulation::Depth\">"
      "<dropout_window>" +
      Window(1, 2) + "</dropout_window><dropout_window>" + Window(4, 6) +
      "</dropout_window></plugin>";
  ASSERT_TRUE(adapter.ConfigureFromPluginXml(xml).ok());
  for (int k = 0; k < 8; ++k) {
    const bool absent = k == 1 || k == 4 || k == 5;
    EXPECT_EQ(adapter.Step(StateAt(k)).measurement.has_value(), !absent) << k;
  }
}

// ---- D6: out of range -----------------------------------------------------

TEST(DepthAdapterTest, AboveSurfaceIsOutOfRange) {
  const std::vector<DepthStepResult> results =
      RunSteps(DefaultChildren(), 2, 0.001);
  for (const DepthStepResult& result : results) {
    ASSERT_TRUE(result.measurement.has_value());
    ExpectOutOfRange(*result.measurement);
    EXPECT_EQ(
        Bytes(result),
        FakeSample(
            result.measurement->health().header().sequence(), 0.0, true,
            TotalNanos(Stamp(result.measurement->health().header(), true)),
            TotalNanos(Stamp(result.measurement->health().header(), false)),
            -0.001)
            .SerializeAsString());
  }
}

TEST(DepthAdapterTest, BelowRatingLimitIsOutOfRange) {
  const Children children = With(DefaultChildren(), "max_depth_m", "50");
  const std::vector<DepthStepResult> results = RunSteps(children, 2, -50.001);
  for (const DepthStepResult& result : results) {
    ASSERT_TRUE(result.measurement.has_value());
    ExpectOutOfRange(*result.measurement);
  }
}

TEST(DepthAdapterTest, BiasAndNoiseAreNotVisibleWhenOutOfRange) {
  const Children children =
      With(With(With(DefaultChildren(), "max_depth_m", "50"), "bias_m", "0.2"),
           "noise_amplitude_m", "0.05");
  for (const double z : {0.001, -50.001}) {
    const std::vector<DepthStepResult> results = RunSteps(children, 20, z);
    for (const DepthStepResult& result : results) {
      ASSERT_TRUE(result.measurement.has_value());
      ExpectOutOfRange(*result.measurement);
    }
  }
}

TEST(DepthAdapterTest, ExactlySurfaceAndExactlyMaxDepthAreInRange) {
  const Children children = With(DefaultChildren(), "max_depth_m", "50");
  for (const double z : {0.0, -50.0}) {
    const std::vector<DepthStepResult> results = RunSteps(children, 2, z);
    for (const DepthStepResult& result : results) {
      ASSERT_TRUE(result.measurement.has_value());
      EXPECT_EQ(Bits(result.measurement->depth_m()), Bits(0.0 - z));
      ExpectAccepted(*result.measurement);
    }
  }
}

TEST(DepthAdapterTest, ReturningToRangeIsValidAgainWithoutHysteresis) {
  const Children children = With(DefaultChildren(), "max_depth_m", "50");
  const std::array<double, 5> zs = {-10.0, 0.001, -10.0, -50.001, -49.0};
  const std::vector<DepthStepResult> results =
      RunSteps(children, 5, [&zs](int k) { return StateAt(k, zs[k]); });
  ASSERT_EQ(results.size(), 5u);
  for (int k = 0; k < 5; ++k) {
    ASSERT_TRUE(results[k].measurement.has_value()) << k;
    if (k == 1 || k == 3) {
      ExpectOutOfRange(*results[k].measurement);
    } else {
      ExpectAccepted(*results[k].measurement);
    }
  }
}

// ---- D7: seed -------------------------------------------------------------

PressureDepthMeasurement WithoutSequence(PressureDepthMeasurement sample) {
  sample.mutable_health()->mutable_header()->clear_sequence();
  return sample;
}

TEST(DepthAdapterTest, SeedWithoutNoiseOnlyChangesTheSequence) {
  const std::vector<DepthStepResult> a =
      RunSteps(With(DefaultChildren(), "seed", "42"), 10);
  const std::vector<DepthStepResult> b =
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

TEST(DepthAdapterTest, SeedWithNoiseOnlyChangesDepthAndSequence) {
  const Children base = With(DefaultChildren(), "noise_amplitude_m", "0.05");
  const std::vector<DepthStepResult> a = RunSteps(With(base, "seed", "42"), 20);
  const std::vector<DepthStepResult> b = RunSteps(With(base, "seed", "43"), 20);
  int differing = 0;
  for (int k = 0; k < 20; ++k) {
    ASSERT_TRUE(a[k].measurement.has_value());
    ASSERT_TRUE(b[k].measurement.has_value());
    PressureDepthMeasurement lhs = WithoutSequence(*a[k].measurement);
    PressureDepthMeasurement rhs = WithoutSequence(*b[k].measurement);
    if (lhs.depth_m() != rhs.depth_m()) {
      ++differing;
    }
    lhs.clear_depth_m();
    rhs.clear_depth_m();
    EXPECT_EQ(lhs.SerializeAsString(), rhs.SerializeAsString()) << k;
  }
  EXPECT_GT(differing, 0);
}

TEST(DepthAdapterTest, MaximumSeedWrapsInsteadOfFailing) {
  const std::vector<DepthStepResult> results =
      RunSteps(With(DefaultChildren(), "seed", "18446744073709551615"), 3);
  ASSERT_TRUE(results[2].measurement.has_value());
  EXPECT_EQ(results[0].measurement->health().header().sequence(),
            std::numeric_limits<uint64_t>::max());
  EXPECT_EQ(results[1].measurement->health().header().sequence(), 0u);
  EXPECT_EQ(results[2].measurement->health().header().sequence(), 1u);
}

// ---- D8: config and state robustness --------------------------------------

DepthSimStatus ConfigureWith(const Children& children) {
  DepthAdapter adapter;
  return adapter.ConfigureFromPluginXml(PluginXml(children));
}

TEST(DepthAdapterTest, InvalidConfigIsRejectedAndNeverClamped) {
  const std::vector<std::pair<std::string, std::string>> bad = {
      {"surface_z_m", "nan"},
      {"surface_z_m", "inf"},
      {"surface_z_m", "-inf"},
      {"surface_z_m", "1.0x"},
      {"surface_z_m", "1,5"},
      {"surface_z_m", ""},
      {"bias_m", "nan"},
      {"bias_m", "inf"},
      {"bias_m", "1.0x"},
      {"bias_m", ""},
      {"noise_amplitude_m", "nan"},
      {"noise_amplitude_m", "inf"},
      {"noise_amplitude_m", "-0.01"},
      {"noise_amplitude_m", "0.05x"},
      {"max_depth_m", "nan"},
      {"max_depth_m", "inf"},
      {"max_depth_m", "0"},
      {"max_depth_m", "-1"},
      {"max_depth_m", "50 m"},
      {"seed", "-1"},
      {"seed", "1.5"},
      {"seed", "abc"},
      {"seed", "18446744073709551616"},
      {"latency_steps", "1001"},
  };
  for (const auto& [name, value] : bad) {
    DepthAdapter adapter;
    const DepthSimStatus status = adapter.ConfigureFromPluginXml(
        PluginXml(With(DefaultChildren(), name, value)));
    EXPECT_EQ(status.code, DepthSimErrorCode::kInvalidConfig)
        << name << "=" << value;
    EXPECT_FALSE(adapter.active());
    for (int k = 0; k < 5; ++k) {
      const DepthStepResult result = adapter.Step(StateAt(k));
      EXPECT_EQ(result.status.code, DepthSimErrorCode::kInvalidConfig);
      EXPECT_FALSE(result.measurement.has_value());
    }
    EXPECT_EQ(adapter.StepIndexForTesting(), 0) << name << "=" << value;
  }
}

TEST(DepthAdapterTest, InvalidDropoutWindowsAreRejected) {
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
    DepthAdapter adapter;
    const std::string xml =
        "<plugin filename=\"static://intrinsic::simulation::Depth\" "
        "name=\"intrinsic::simulation::Depth\"><dropout_window>" +
        window + "</dropout_window></plugin>";
    EXPECT_EQ(adapter.ConfigureFromPluginXml(xml).code,
              DepthSimErrorCode::kInvalidConfig)
        << window;
    EXPECT_FALSE(adapter.Step(StateAt(0)).measurement.has_value());
  }
}

TEST(DepthAdapterTest, MoreThanSixtyFourWindowsAreRejected) {
  for (const int count : {64, 65}) {
    std::string xml =
        "<plugin filename=\"static://intrinsic::simulation::Depth\" "
        "name=\"intrinsic::simulation::Depth\">";
    for (int i = 0; i < count; ++i) {
      xml +=
          "<dropout_window>" + Window(2 * i, 2 * i + 1) + "</dropout_window>";
    }
    xml += "</plugin>";
    DepthAdapter adapter;
    const DepthSimStatus status = adapter.ConfigureFromPluginXml(xml);
    if (count == 64) {
      EXPECT_TRUE(status.ok()) << status.message;
    } else {
      EXPECT_EQ(status.code, DepthSimErrorCode::kInvalidConfig);
      EXPECT_FALSE(adapter.Step(StateAt(0)).measurement.has_value());
    }
  }
}

TEST(DepthAdapterTest, UnknownChildrenAreIgnoredAndAllOptionsMayBeUnset) {
  EXPECT_TRUE(ConfigureWith({{"made_up", "value"}}).ok());
  EXPECT_TRUE(ConfigureWith({}).ok());
  EXPECT_TRUE(ConfigureWith({{"link_name", "hull"}}).ok());
}

TEST(DepthAdapterTest, UnparseableOrUnconfiguredAdapterFailsClosed) {
  DepthAdapter unconfigured;
  EXPECT_FALSE(unconfigured.active());
  const DepthStepResult result = unconfigured.Step(StateAt(0));
  EXPECT_EQ(result.status.code, DepthSimErrorCode::kInvalidConfig);
  EXPECT_FALSE(result.measurement.has_value());

  DepthAdapter garbage;
  EXPECT_EQ(garbage.ConfigureFromPluginXml("<not-a-plugin/>").code,
            DepthSimErrorCode::kInvalidConfig);
  EXPECT_FALSE(garbage.Step(StateAt(0)).measurement.has_value());
}

TEST(DepthAdapterTest, ReconfigureReplacesAnEarlierBadConfiguration) {
  DepthAdapter adapter;
  ASSERT_FALSE(
      adapter.ConfigureFromPluginXml(PluginXml({{"bias_m", "nan"}})).ok());
  ASSERT_TRUE(adapter.ConfigureFromPluginXml(PluginXml({})).ok());
  EXPECT_TRUE(adapter.active());
  EXPECT_TRUE(adapter.Step(StateAt(0)).measurement.has_value());
}

TEST(DepthAdapterTest, InvalidStateGivesNoSampleAndStillAdvancesTheStep) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  std::vector<DepthSimulatorState> bad;
  DepthSimulatorState state = StateAt(1);
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

  for (const DepthSimulatorState& bad_state : bad) {
    DepthAdapter adapter;
    ASSERT_TRUE(adapter.ConfigureFromPluginXml(PluginXml({})).ok());
    EXPECT_TRUE(adapter.Step(StateAt(0)).measurement.has_value());
    const DepthStepResult result = adapter.Step(bad_state);
    EXPECT_EQ(result.status.code, DepthSimErrorCode::kInvalidState);
    EXPECT_FALSE(result.measurement.has_value());
    EXPECT_EQ(result.step, 1);
    EXPECT_EQ(adapter.StepIndexForTesting(), 2);
    EXPECT_TRUE(adapter.active());
    const DepthStepResult recovered = adapter.Step(StateAt(2));
    EXPECT_TRUE(recovered.status.ok());
    ASSERT_TRUE(recovered.measurement.has_value());
    EXPECT_EQ(recovered.step, 2);
    ExpectAccepted(*recovered.measurement);
  }
}

TEST(DepthAdapterTest, QuaternionWithinToleranceIsAcceptedAndNeverNormalised) {
  DepthAdapter adapter;
  ASSERT_TRUE(adapter.ConfigureFromPluginXml(PluginXml({})).ok());
  DepthSimulatorState state = StateAt(0);
  state.orientation_xyzw = {0, 0, 0, 1.0 + 5e-7};
  const DepthStepResult result = adapter.Step(state);
  EXPECT_TRUE(result.status.ok());
  EXPECT_TRUE(result.measurement.has_value());
}

TEST(DepthAdapterTest, NonUnitQuaternionCostsTheDelayedSampleToo) {
  const Children children = With(DefaultChildren(), "latency_steps", "2");
  DepthAdapter adapter;
  ASSERT_TRUE(adapter.ConfigureFromPluginXml(PluginXml(children)).ok());
  std::vector<bool> present;
  for (int k = 0; k < 8; ++k) {
    DepthSimulatorState state = StateAt(k);
    if (k == 3) {
      state.orientation_xyzw = {0, 0, 0, 1.0 + 2e-6};
    }
    present.push_back(adapter.Step(state).measurement.has_value());
  }
  // Startup 0-1 absent, step 3 bad, and the sample originating at 3 would be
  // emitted at 5.
  const std::vector<bool> expected = {false, false, true, false,
                                      true,  false, true, true};
  EXPECT_EQ(present, expected);
}

// ---- Determinism, reset ---------------------------------------------------

TEST(DepthAdapterTest, TwoFreshAdaptersAreByteIdenticalOverAThousandSteps) {
  const Children children =
      With(With(With(DefaultChildren(), "noise_amplitude_m", "0.05"), "bias_m",
                "0.01"),
           "latency_steps", "5");
  const StateFn state_fn = [](int k) { return StateAt(k, -5.0 - 0.01 * k); };
  const std::vector<DepthStepResult> first = RunSteps(children, 1000, state_fn);
  const std::vector<DepthStepResult> second =
      RunSteps(children, 1000, state_fn);
  for (int k = 0; k < 1000; ++k) {
    ASSERT_EQ(first[k].measurement.has_value(),
              second[k].measurement.has_value());
    if (first[k].measurement.has_value()) {
      EXPECT_EQ(Bytes(first[k]), Bytes(second[k])) << k;
    }
  }
}

TEST(DepthAdapterTest, ResetThenRerunReproducesTheSameTrace) {
  const Children children =
      With(With(With(DefaultChildren(), "noise_amplitude_m", "0.05"),
                "latency_steps", "3"),
           "dropout_window", Window(4, 6));
  DepthAdapter adapter;
  ASSERT_TRUE(adapter.ConfigureFromPluginXml(PluginXml(children)).ok());
  std::vector<std::optional<std::string>> first;
  for (int k = 0; k < 30; ++k) {
    const DepthStepResult result = adapter.Step(StateAt(k));
    first.push_back(result.measurement.has_value()
                        ? std::optional<std::string>(Bytes(result))
                        : std::nullopt);
  }
  EXPECT_EQ(adapter.StepIndexForTesting(), 30);
  adapter.Reset();
  EXPECT_EQ(adapter.StepIndexForTesting(), 0);
  for (int k = 0; k < 30; ++k) {
    const DepthStepResult result = adapter.Step(StateAt(k));
    EXPECT_EQ(result.step, k);
    if (first[k].has_value()) {
      ASSERT_TRUE(result.measurement.has_value()) << k;
      EXPECT_EQ(Bytes(result), *first[k]) << k;
    } else {
      EXPECT_FALSE(result.measurement.has_value()) << k;
    }
  }
}

TEST(DepthAdapterTest, ResetClearsPendingLatencyQueue) {
  const Children children = With(DefaultChildren(), "latency_steps", "3");
  DepthAdapter adapter;
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

// ---- D9: golden -----------------------------------------------------------

Children GoldenChildren() {
  Children children = DefaultChildren();
  children = With(children, "max_depth_m", "4");
  children = With(children, "bias_m", "0.01");
  children = With(children, "noise_amplitude_m", "0.02");
  children = With(children, "latency_steps", "2");
  children = With(children, "dropout_window", Window(5, 7));
  children = With(children, "seed", "42");
  return children;
}

std::vector<DepthStepResult> GoldenRun() {
  return RunSteps(GoldenChildren(), 20,
                  [](int k) { return StateAt(k, -3.0 + 0.25 * k); });
}

TEST(DepthAdapterTest, GoldenSeed42Trace) {
  const std::vector<DepthStepResult> results = GoldenRun();
  std::string trace = "depth_sim_trace v1\nseed=42\n";
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

TEST(DepthAdapterTest, GoldenScenarioShapeIsAsContracted) {
  const std::vector<DepthStepResult> results = GoldenRun();
  for (int k = 0; k < 20; ++k) {
    const bool expect_absent = k < 2 || (k >= 7 && k < 9);
    ASSERT_EQ(results[k].measurement.has_value(), !expect_absent) << k;
    if (expect_absent) {
      continue;
    }
    const int origin = k - 2;
    // d = 3 - 0.25 origin: in range for origin 0..12 (d == 0 at 12), out of
    // range from origin 13.
    if (origin >= 13) {
      ExpectOutOfRange(*results[k].measurement);
    } else {
      const double effective = std::fma(0.02, Unit(42, origin), 0.01);
      const double depth = results[k].measurement->depth_m();
      EXPECT_EQ(Bits(depth), Bits((3.0 - 0.25 * origin) + effective)) << origin;
      if (depth < 0.0) {
        // Noise pushed the depth below 0 at the surface crossing. The adapter
        // does not repair it, so the oracle reports kDepth.
        ExpectOracle(*results[k].measurement, PressureDepthError::kDepth,
                     MeasurementStateKind::kDegraded, false);
        continue;
      }
      ExpectDegradedNotAccepted(*results[k].measurement);
    }
  }
}

}  // namespace
}  // namespace intrinsic::simulation
